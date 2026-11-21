"""Reference baselines: (1) 'stand still' and (2) naive strapdown double integration."""
import os, sys, tempfile, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from score import score
from bench import TRAIN, TEST

def load_imu(d):
    a = np.genfromtxt(os.path.join(d, "mav0/imu0/data.csv"), delimiter=",", comments="#")
    return a

def write(path, t, p, q=None):
    with open(path, "w") as f:
        for i in range(len(t)):
            qq = q[i] if q is not None else (0, 0, 0, 1)
            f.write(f"{t[i]:.9f} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f} {qq[0]} {qq[1]} {qq[2]} {qq[3]}\n")

def times(a, d):
    import json; meta = json.load(open(os.path.join(d, "meta.json")))
    t = a[:, 0] * 1e-9
    good = np.isfinite(a[:, 1:]).all(1) & (np.abs(a[:, 1:]).sum(1) > 0)
    last = np.nonzero(good)[0][-1]
    tt = list(t[:last + 1]); dt = 0.005
    tt += list(t[last] + dt * np.arange(1, 1001))
    return np.array(tt), a[:last + 1], good[:last + 1]

def naive(a, good, tt):
    t = a[:, 0] * 1e-9
    w, f = a[:, 1:4], a[:, 4:7]
    am = f[:200][good[:200]].mean(0); u = am / np.linalg.norm(am); z = np.array([0, 0, 1.])
    ax = np.cross(u, z); s = np.linalg.norm(ax); th = np.arctan2(s, u @ z)
    K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]]) / s
    R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    p = np.zeros(3); v = np.zeros(3); P = []
    g = np.linalg.norm(am)
    for i in range(len(t)):
        P.append(p.copy())
        if i + 1 == len(t): break
        dt = t[i + 1] - t[i]
        if good[i]:
            acc = R @ f[i] - [0, 0, g]
            th = w[i] * dt; n = np.linalg.norm(th)
            if n > 1e-12:
                k = th / n; Kk = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
                R = R @ (np.eye(3) + np.sin(n) * Kk + (1 - np.cos(n)) * Kk @ Kk)
        else:
            acc = np.zeros(3)
        p = p + v * dt + 0.5 * acc * dt * dt; v = v + acc * dt
    P = np.array(P)
    ext = P[-1] + np.outer(np.arange(1, 1001) * 0.005, v)
    return np.vstack([P, ext])

def main(seqs):
    res = {"still": [], "naive": []}
    with tempfile.TemporaryDirectory() as td:
        for seq in seqs:
            for st in ("missing", "zeros"):
                d = f"data/dev/{seq}/{st}"; a = load_imu(d); tt, aa, good = times(a, d)
                o = os.path.join(td, "o.txt")
                write(o, tt, np.zeros((len(tt), 3))); res["still"].append(score(o, d))
                write(o, tt, naive(aa, good, tt)); res["naive"].append(score(o, d))
    for k, rows in res.items():
        print(k.ljust(8), "  ".join(f"{m}={np.mean([r[m] for r in rows]):.2f}" for m in ("ate", "wp", "drop", "blind", "rpe1s")))

if __name__ == "__main__":
    split = sys.argv[1] if len(sys.argv) > 1 else "all"
    main({"train": TRAIN, "test": TEST, "all": TRAIN + TEST}[split])
