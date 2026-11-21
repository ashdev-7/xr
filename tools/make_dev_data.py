"""Build practice inputs for Track 3 from EuRoC ground truth.

The organizers' raw IMU logs were not reachable from our build machine, so we
synthesize EuRoC-format IMU from the official ground truth (pose, velocity and
the dataset's own estimated sensor biases), add ADIS16448-like noise and
propeller vibration, and then apply a degradation that follows the brief:
progressive noise, bias drift, 2 s dropouts, spikes, and a final IMU failure
followed by a 5 s blind window.

Outputs per sequence (in data/dev/<seq>/<variant>/):
  mav0/imu0/data.csv          degraded IMU, EuRoC header and units
  groundtruth.txt              TUM-format truth for every IMU timestamp + blind 5 s
  meta.json                    where dropouts / failure happened (for scoring only)
"""
import argparse, json, os, zlib
import numpy as np

G = 9.81
HDR = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")


def q2R(q):  # q = w,x,y,z (N,4)
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def log_so3(R):
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    th = np.arccos(c)
    s = np.where(th < 1e-8, 1.0, th / (2 * np.sin(np.maximum(th, 1e-12))))
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], 1)
    return v * s[:, None]


def smooth(x, k):
    if k <= 1:
        return x
    ker = np.ones(k) / k
    pad = np.pad(x, ((k // 2, k - 1 - k // 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(pad[:, i], ker, mode="valid") for i in range(x.shape[1])], 1)


def clean_imu(gt):
    """Ideal IMU (body-frame gyro and specific force) from ground truth."""
    t = gt[:, 0] * 1e-9
    p, q, v = gt[:, 1:4], gt[:, 4:8], gt[:, 8:11]
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    R = q2R(q)
    dt = np.diff(t)
    # gyro: body-frame rotation between consecutive poses
    dR = np.einsum("nji,njk->nik", R[:-1], R[1:])
    w = log_so3(dR) / dt[:, None]
    w = np.vstack([w, w[-1:]])
    # acceleration: differentiate velocity (central), light smoothing
    a_w = np.gradient(smooth(v, 3), t, axis=0)
    a_w = smooth(a_w, 5)
    w = smooth(w, 3)
    f_b = np.einsum("nji,nj->ni", R, a_w + np.array([0, 0, G]))
    return t, p, q, w, f_b, gt[:, 11:14], gt[:, 14:17]


def degrade(t, w, f, bw, ba, rng, n_drop=3, drop_style="missing", blind=5.0, severity=1.0):
    n = len(t); dt = np.median(np.diff(t)); fs = 1 / dt
    T = t - t[0]; frac = T / T[-1]
    # base sensor errors: dataset biases + white noise (ADIS16448 datasheet-ish)
    gn, an = 1.7e-4 * np.sqrt(fs), 2.0e-3 * np.sqrt(fs)
    w_m = w + bw + rng.normal(0, gn, (n, 3))
    f_m = f + ba + rng.normal(0, an, (n, 3))
    # propeller vibration while airborne (dominant real-world accel noise on EuRoC)
    vib = smooth(rng.normal(0, 1, (n, 3)), 2) * 0.35
    f_m += vib
    # progressive jamming: noise grows up to 6x, biases random-walk faster over time
    scale = 1 + 5 * severity * frac ** 1.5
    w_m += rng.normal(0, 1, (n, 3)) * gn * (scale[:, None] - 1) * 3
    f_m += rng.normal(0, 1, (n, 3)) * an * (scale[:, None] - 1) * 3
    bw_drift = np.cumsum(rng.normal(0, 2e-5 * severity, (n, 3)), 0) * (1 + 4 * frac[:, None])
    ba_drift = np.cumsum(rng.normal(0, 2e-4 * severity, (n, 3)), 0) * (1 + 4 * frac[:, None])
    w_m += bw_drift; f_m += ba_drift
    # spikes
    k = rng.choice(n, int(0.002 * n * severity), replace=False)
    w_m[k] += rng.normal(0, 3, (len(k), 3)); f_m[k] += rng.normal(0, 40, (len(k), 3))
    # final failure: IMU dies blind seconds before the end of truth
    fail_idx = int(np.searchsorted(t, t[-1] - blind))
    # dropouts: 2 s gaps in the flying part, not overlapping, before failure
    valid = np.ones(n, bool); drops = []
    lo, hi = int(0.15 * n), fail_idx - int(4 * fs)
    starts = np.sort(rng.choice(np.arange(lo, hi, int(2 * fs)), n_drop, replace=False))
    for s in starts:
        e = s + int(round(2 * fs)); valid[s:e] = False; drops.append([float(t[s]), float(t[e - 1])])
    return w_m, f_m, valid, fail_idx, drops


def write(out, ts_ns, w_m, f_m, valid, fail_idx, drop_style, gt_t, gt_p, gt_q, meta):
    os.makedirs(os.path.join(out, "mav0", "imu0"), exist_ok=True)
    rows = []
    for i in range(fail_idx):
        if not valid[i]:
            if drop_style == "missing":
                continue
            if drop_style == "zeros":
                rows.append(f"{ts_ns[i]},0,0,0,0,0,0"); continue
            if drop_style == "nan":
                rows.append(f"{ts_ns[i]},nan,nan,nan,nan,nan,nan"); continue
        rows.append(f"{ts_ns[i]}," + ",".join(f"{x:.9g}" for x in (*w_m[i], *f_m[i])))
    if drop_style != "missing":  # failed sensor keeps emitting dead rows until the end
        dead = "0,0,0,0,0,0" if drop_style == "zeros" else "nan,nan,nan,nan,nan,nan"
        rows += [f"{ts_ns[i]},{dead}" for i in range(fail_idx, len(ts_ns))]
    with open(os.path.join(out, "mav0", "imu0", "data.csv"), "w") as fh:
        fh.write(HDR + "\n" + "\n".join(rows) + "\n")
    with open(os.path.join(out, "groundtruth.txt"), "w") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(gt_t)):
            w, x, y, z = gt_q[i]
            fh.write(f"{gt_t[i]:.9f} {gt_p[i,0]:.6f} {gt_p[i,1]:.6f} {gt_p[i,2]:.6f} {x:.7f} {y:.7f} {z:.7f} {w:.7f}\n")
    with open(os.path.join(out, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default="data/gt")
    ap.add_argument("--out", default="data/dev")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    for fn in sorted(os.listdir(a.gt)):
        if not fn.endswith(".csv") or fn.endswith(".bak"):
            continue
        seq = fn[:-4]
        gt = np.loadtxt(os.path.join(a.gt, fn), delimiter=",", comments="#")
        t, p, q, w, f, bw, ba = clean_imu(gt)
        ts_ns = gt[:, 0].astype(np.int64)
        for style, sev in (("missing", 1.0), ("zeros", 1.5)):
            rng = np.random.default_rng(zlib.crc32(f"{seq}/{style}/{a.seed}".encode()))
            w_m, f_m, valid, fail_idx, drops = degrade(t, w, f, bw, ba, rng, severity=sev)
            meta = dict(seq=seq, drop_style=style, severity=sev, dropouts=drops,
                        fail_time=float(t[fail_idx]), end_time=float(t[-1]))
            write(os.path.join(a.out, seq, style), ts_ns, w_m, f_m, valid, fail_idx, style, t, p, q, meta)
        print(seq, f"{t[-1]-t[0]:.0f}s", "fail at", f"{t[fail_idx]-t[0]:.1f}s")


if __name__ == "__main__":
    main()
