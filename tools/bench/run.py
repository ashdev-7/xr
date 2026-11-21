"""Run a method over benchmark variants in parallel and print a score table.

usage: python tools/bench/run.py [--method pg|still|naive|cpp] [--flights A,B] [--scen asis,aware]
                                 [--seeds 42,1,2] [--tag name] [--param=value ...]
Methods
  pg     src/main.py (the submission) with --param overrides
  still  stand-still baseline: every position 0, identity attitude
  naive  strapdown double integration from the first calm second (no bias/prior handling)
  cpp    legacy C++ v1 (legacy/main.cpp, built to build/pathguard) with --append_blind=0
"""
import json, os, subprocess, sys, time
from concurrent.futures import ProcessPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from score import score, read_tum  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BENCH = os.path.join(ROOT, "data", "bench")
PY = sys.executable if os.name != "nt" else "python"
KEYS = ["ate", "wp", "wp10", "blind", "drop", "rpe_t", "rpe_r", "are"]


def variants(flights=None, scen=("asis", "aware"), seeds=(42,)):
    out = []
    for fl in sorted(os.listdir(BENCH)):
        if flights and fl not in flights:
            continue
        for sc in scen:
            for sd in seeds:
                d = os.path.join(BENCH, fl, f"{sc}_s{sd}")
                if os.path.isdir(d):
                    out.append((fl, sc, sd, d))
    return out


def load_input(d):
    rows = []
    with open(os.path.join(d, "imu0", "data.csv")) as f:
        for line in f:
            if line[0].isdigit():
                rows.append([float(x) for x in line.split(",")])
    a = np.array(rows)
    ts = [line.split(",")[0] for line in open(os.path.join(d, "imu0", "data.csv")) if line[0].isdigit()]
    return a, ts


def fmt_ts(tok):
    return tok[:-9] + "." + tok[-9:]


def write_traj(path, ts, p, q=None):
    with open(path, "w") as f:
        for i, tok in enumerate(ts):
            qq = q[i] if q is not None else (0, 0, 0, 1)
            f.write(f"{fmt_ts(tok)} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f} {qq[0]:.9f} {qq[1]:.9f} {qq[2]:.9f} {qq[3]:.9f}\n")


def m_still(d, out):
    a, ts = load_input(d)
    write_traj(out, ts, np.zeros((len(ts), 3)))


def m_naive(d, out):
    a, ts = load_input(d)
    t = a[:, 0] * 1e-9; w = a[:, 1:4]; f = a[:, 4:7]
    good = np.isfinite(a[:, 1:]).all(1) & (np.abs(a[:, 1:]).sum(1) > 0)
    am = f[:200][good[:200]].mean(0); u = am / np.linalg.norm(am)
    ax = np.cross(u, [0, 0, 1.0]); s = np.linalg.norm(ax); th = np.arctan2(s, u[2])
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]) / s
    R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    g = np.linalg.norm(am); p = np.zeros(3); v = np.zeros(3); P = np.zeros((len(t), 3))
    for i in range(len(t)):
        P[i] = p
        if i + 1 == len(t):
            break
        dt = t[i + 1] - t[i]
        if good[i]:
            acc = R @ f[i] - [0, 0, g]
            th = w[i] * dt; n = np.linalg.norm(th)
            if n > 1e-12:
                k = th / n; Kk = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                R = R @ (np.eye(3) + np.sin(n) * Kk + (1 - np.cos(n)) * Kk @ Kk)
        else:
            acc = np.zeros(3)
        p = p + v * dt + 0.5 * acc * dt * dt; v = v + acc * dt
    write_traj(out, ts, P)


def job(args):
    method, fl, sc, sd, d, params, outdir = args
    out = os.path.join(outdir, f"{fl}__{sc}_s{sd}.txt")
    t0 = time.perf_counter()
    if method == "pg":
        r = subprocess.run([PY, "-I", os.path.join(ROOT, "src", "main.py"), d, out, *params], capture_output=True, text=True)
        if r.returncode:
            return dict(flight=fl, scen=sc, seed=sd, error=r.stderr[-2000:])
    elif method == "cpp":
        exe = os.path.join(ROOT, "build", "pathguard.exe" if os.name == "nt" else "pathguard")
        r = subprocess.run([exe, os.path.join(d, "imu0", "data.csv"), out, "--append_blind=0", *params], capture_output=True, text=True)
        if r.returncode:
            return dict(flight=fl, scen=sc, seed=sd, error=r.stderr[-2000:])
    elif method.startswith("oracle:"):
        r = subprocess.run([sys.executable, os.path.join(HERE, "oracle.py"), method.split(":")[1], d, out, *params], capture_output=True, text=True)
        if r.returncode:
            return dict(flight=fl, scen=sc, seed=sd, error=r.stderr[-2000:])
    elif method == "still":
        m_still(d, out)
    elif method == "naive":
        m_naive(d, out)
    wall = time.perf_counter() - t0
    s = score(out, d)
    meta = json.load(open(os.path.join(d, "meta.json")))
    s.update(flight=fl, scen=sc, seed=sd, wall=wall, dur=meta["t_last"] - meta["t_first"])
    return s


def run(method="pg", params=(), flights=None, scen=("asis", "aware"), seeds=(42,), tag=None, quiet=False, workers=10):
    outdir = os.path.join(ROOT, "results", tag or method)
    os.makedirs(outdir, exist_ok=True)
    jobs = [(method, fl, sc, sd, d, list(params), outdir) for fl, sc, sd, d in variants(flights, scen, seeds)]
    with ProcessPoolExecutor(workers) as ex:
        res = list(ex.map(job, jobs))
    errs = [r for r in res if "error" in r]
    for e in errs:
        print("ERROR", e["flight"], e["scen"], e["seed"], e["error"])
    res = [r for r in res if "error" not in r]
    if not quiet:
        print(f"{'variant':34s}" + "".join(f"{k:>8s}" for k in KEYS) + f"{'ms/s':>7s}  issues")
        for r in res:
            print(f"{r['flight'] + '/' + r['scen'] + '_s' + str(r['seed']):34s}" + "".join(f"{r[k]:8.3f}" for k in KEYS)
                  + f"{1000 * r['wall'] / r['dur']:7.2f}  {'; '.join(r['issues'])}")
    mean = {k: float(np.nanmean([r[k] for r in res])) for k in KEYS}
    mean["ms_per_s"] = float(np.mean([1000 * r["wall"] / r["dur"] for r in res])) if res else float("nan")
    print(f"{'MEAN ' + (tag or method):34s}" + "".join(f"{mean[k]:8.3f}" for k in KEYS) + f"{mean['ms_per_s']:7.2f}"
          + f"   (n={len(res)}, issues={sum(1 for r in res if r['issues'])})")
    return mean, res


if __name__ == "__main__":
    a = sys.argv[1:]
    method, flights, scen, seeds, tag, params = "pg", None, ("asis", "aware"), (42,), None, []
    while a:
        x = a.pop(0)
        if x == "--method": method = a.pop(0)
        elif x == "--flights": flights = a.pop(0).split(",")
        elif x == "--scen": scen = tuple(a.pop(0).split(","))
        elif x == "--seeds": seeds = tuple(int(s) for s in a.pop(0).split(","))
        elif x == "--tag": tag = a.pop(0)
        else: params.append(x)
    run(method, params, flights, scen, seeds, tag)
