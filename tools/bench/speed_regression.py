"""Offline feasibility test for a learned speed measurement (RIDI / Cortes 2018 / AirIO idea, ridge version).

Features are heading-independent statistics of 1 s windows of the DEGRADED IMU (official script, aware and asis);
labels are ground-truth horizontal speed over the same window. Leave-one-flight-out ridge regression.
Pass criterion (set before running): correlation r >= 0.6 and RMSE <= 35 % (drone) / 25 % (handheld) of mean speed.
usage: python tools/bench/speed_regression.py
"""
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from score import read_tum  # noqa: E402

BENCH = os.path.join(HERE, "..", "..", "data", "bench")
GROUPS = {"drone": ["MH_01_easy", "MH_03_medium", "MH_05_difficult", "V1_01_easy", "V2_02_medium"],
          "handheld": ["TUMVI_room1", "TUMVI_room3", "TUMVI_room5"]}


def features(a, t, drone):
    w, f = a[:, 1:4], a[:, 4:7]
    ok = (np.abs(a[:, 1:]).sum(1) > 0)
    t, w, f = t[ok], w[ok], f[ok]
    u = f.mean(0); u /= np.linalg.norm(u)                       # thrust axis / mean up direction (no GT)
    fp = f - np.outer(f @ u, u)                                 # rotor-plane / horizontal-ish specific force
    sec = np.floor(t - t[0]).astype(int)
    n = sec.max() + 1
    X = np.full((n, 8), np.nan)
    for s in range(n):
        m = sec == s
        if m.sum() < 60:
            continue
        fs, ws, fps = f[m], w[m], fp[m]
        hp = fs - fs.mean(0)
        X[s] = [np.linalg.norm(fps.mean(0)),                    # mean rotor-plane force (drag ~ mu * speed)
                np.sqrt(np.mean(np.sum(hp ** 2, 1))),           # accel energy (vibration / steps)
                np.sqrt(np.mean(np.sum(ws ** 2, 1))),           # gyro energy
                np.linalg.norm(ws.mean(0)),                     # mean turn rate
                abs((fs @ u).mean() - 9.81),                    # thrust deviation (accelerating / climbing)
                np.abs(np.diff(fs @ u)).mean(),                 # thrust-axis roughness
                0.0, 0.0]
    # multi-second context: 5 s averages of the first two features
    for k, src in ((6, 0), (7, 1)):
        col = X[:, src]
        X[:, k] = np.array([np.nanmean(col[max(0, i - 2):i + 3]) for i in range(n)])
    return X, t[0]


def labels(fl, t0, n):
    tg, pg, _ = read_tum(os.path.join(BENCH, fl, "gt.tum"))
    y = np.full(n, np.nan)
    for s in range(n):
        a, b = np.searchsorted(tg, [t0 + s, t0 + s + 1])
        if b - a > 10:
            b -= 1
            y[s] = np.linalg.norm(pg[b, :2] - pg[a, :2]) / (tg[b] - tg[a])
    return y


def main():
    for grp, fls in GROUPS.items():
        data = {}
        for fl in fls:
            Xs, ys = [], []
            for sc in ("aware_s42", "asis_s42", "aware_s1", "asis_s1"):
                a = np.loadtxt(os.path.join(BENCH, fl, sc, "imu0", "data.csv"), delimiter=",", comments="#")
                t = a[:, 0] * 1e-9
                X, t0 = features(a, t, grp == "drone")
                y = labels(fl, t0, len(X))
                n = len(X) - 6  # drop the failure window
                Xs.append(X[:n]); ys.append(y[:n])
            X, y = np.vstack(Xs), np.concatenate(ys)
            m = np.isfinite(X).all(1) & np.isfinite(y)
            data[fl] = (X[m], y[m])
        print(f"== {grp}: leave-one-flight-out ridge regression of 1 s horizontal speed")
        allr = []
        for fl in fls:
            Xtr = np.vstack([data[g][0] for g in fls if g != fl]); ytr = np.concatenate([data[g][1] for g in fls if g != fl])
            Xte, yte = data[fl]
            mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-9
            A = np.hstack([(Xtr - mu) / sd, np.ones((len(Xtr), 1))])
            wgt = np.linalg.solve(A.T @ A + 1.0 * np.eye(A.shape[1]), A.T @ ytr)
            pred = np.hstack([(Xte - mu) / sd, np.ones((len(Xte), 1))]) @ wgt
            r = np.corrcoef(pred, yte)[0, 1]
            rmse = np.sqrt(np.mean((pred - yte) ** 2))
            const = np.sqrt(np.mean((ytr.mean() - yte) ** 2))
            allr.append(r)
            print(f"  {fl:16s} r={r:5.2f}  RMSE={rmse:.3f} m/s ({100 * rmse / yte.mean():4.0f}% of mean {yte.mean():.2f})"
                  f"  constant-speed RMSE={const:.3f}  flight mean: true {yte.mean():.2f} predicted {pred.mean():.2f}")
        print(f"  mean r = {np.mean(allr):.2f}")


if __name__ == "__main__":
    main()
