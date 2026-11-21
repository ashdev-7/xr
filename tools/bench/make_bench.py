"""Build the evaluation benchmark from REAL IMU recordings with the OFFICIAL degradation script.

For every flight (EuRoC MAV: imu0 + state_groundtruth_estimate0; TUM-VI room: imu0 + mocap0):

  data/bench/<flight>/gt.tum                      ground truth, TUM format (t[s] x y z qx qy qz qw)
  data/bench/<flight>/<scenario>_s<seed>/imu0/data.csv   degraded IMU, exactly as the organizers would produce it
  data/bench/<flight>/<scenario>_s<seed>/meta.json       script stats + gap list (used by the scorer only)

Scenarios (we do not know which one the organizers used, so we test both):
  asis   official script run on the EuRoC-order file as is. The script assumes columns
         (t, accel, gyro) but EuRoC is (t, gyro, accel): gyro receives the accel-sized
         degradation and vice versa. This is what running the script on imu0/data.csv does.
  aware  columns reordered to (t, accel, gyro) before the script, and back afterwards, so each
         sensor gets its intended degradation.

The script is executed unmodified (imported as a module, SEED overridden for extra seeds only).
"""
import argparse, io, json, os, sys
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import degrade_imu_official as official  # noqa: E402

RAW = "data/raw"
OUT = "data/bench"
SEEDS = (42, 1, 2)


def load_flights():
    fl = {}
    for root, kind in ((os.path.join(RAW, "euroc"), "euroc"), (os.path.join(RAW, "tumvi"), "tumvi")):
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            d = os.path.join(root, name)
            imu = os.path.join(d, "imu0.csv")
            gt = os.path.join(d, "gt.csv" if kind == "euroc" else "mocap0.csv")
            if os.path.isfile(imu) and os.path.isfile(gt):
                fl[(name if kind == "euroc" else "TUMVI_" + name)] = (kind, imu, gt)
    return fl


def write_gt(kind, gt_path, out_path):
    g = np.loadtxt(gt_path, delimiter=",", comments="#")
    t_ns = g[:, 0].astype(np.int64)
    p = g[:, 1:4]
    qw, qx, qy, qz = g[:, 4], g[:, 5], g[:, 6], g[:, 7]
    with open(out_path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(g)):
            s = f"{t_ns[i] // 10**9}.{t_ns[i] % 10**9:09d}"
            f.write(f"{s} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f} {qx[i]:.9f} {qy[i]:.9f} {qz[i]:.9f} {qw[i]:.9f}\n")
    return t_ns


LEVEL = None  # None = official default max_level


def run_official(df, seed):
    """Replicates degrade_imu.main() for an in-memory dataframe (header present)."""
    cfg = dict(official.CONFIG)
    if LEVEL is not None:
        cfg["max_level"] = LEVEL
    out, stats = official.degrade(df, cfg, seed)
    return out, stats


def gaps(ts_ns, dt_nom):
    t = ts_ns.astype(np.float64) * 1e-9
    d = np.diff(t)
    k = np.nonzero(d > 1.5 * dt_nom)[0]
    return [[float(t[i]), float(t[i + 1]), int(round(d[i] / dt_nom)) - 1] for i in k]


def build(flight, kind, imu_path, gt_path, seeds, scenarios):
    od = os.path.join(OUT, flight)
    os.makedirs(od, exist_ok=True)
    gt_t = write_gt(kind, gt_path, os.path.join(od, "gt.tum"))
    header = official.detect_header(imu_path)
    raw = pd.read_csv(imu_path, header=header)
    assert raw.shape[1] == 7, raw.shape
    cols = list(raw.columns)  # t, w xyz, a xyz (EuRoC order)
    dt_nom = float(np.median(np.diff(raw[cols[0]].to_numpy(np.int64)))) * 1e-9
    for sc in scenarios:
        for seed in seeds:
            if sc == "asis":
                out, stats = run_official(raw, seed)
            else:  # aware: present accel first, then restore EuRoC order
                re = raw[[cols[0]] + cols[4:7] + cols[1:4]]
                out, stats = run_official(re, seed)
                out = out[cols]
            vd = os.path.join(od, f"{sc}{'' if LEVEL is None else '_L%g' % LEVEL}_s{seed}")
            os.makedirs(os.path.join(vd, "imu0"), exist_ok=True)
            out.to_csv(os.path.join(vd, "imu0", "data.csv"), index=False, header=True)
            ts = out[cols[0]].to_numpy(np.int64)
            t0, tN = ts[0] * 1e-9, ts[-1] * 1e-9
            meta = dict(flight=flight, kind=kind, scenario=sc, seed=seed, dt_nom=dt_nom,
                        t_first=t0, t_last=tN, fail_time=tN - official.CONFIG["failure_seconds"],
                        rows=int(len(out)), gaps=gaps(ts, dt_nom),
                        gt_first=float(gt_t[0] * 1e-9), gt_last=float(gt_t[-1] * 1e-9),
                        stats={k: v for k, v in stats.items() if k != "config"})
            with open(os.path.join(vd, "meta.json"), "w") as f:
                json.dump(meta, f, indent=1)
    print(f"{flight:22s} {kind:6s} rows={len(raw):6d} dur={(raw[cols[0]].iloc[-1]-raw[cols[0]].iloc[0])*1e-9:6.1f}s "
          f"gt=[{(gt_t[0]-raw[cols[0]].iloc[0])*1e-9:+.2f}s, {(gt_t[-1]-raw[cols[0]].iloc[-1])*1e-9:+.2f}s]", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default=",".join(map(str, SEEDS)))
    ap.add_argument("--scenarios", default="asis,aware")
    ap.add_argument("--level", type=float, default=None, help="override max_level (oracle/stress variants)")
    ap.add_argument("flights", nargs="*")
    a = ap.parse_args()
    LEVEL = a.level
    fl = load_flights()
    for name, (kind, imu, gt) in fl.items():
        if a.flights and name not in a.flights:
            continue
        build(name, kind, imu, gt, [int(s) for s in a.seeds.split(",")], a.scenarios.split(","))
