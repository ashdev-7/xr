"""Figures for the deck / walkthrough video."""
import os, sys, json, subprocess, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(__file__))
from score import load_tum, associate, umeyama
from baselines import load_imu, times, naive
plt.rcParams.update({"font.size": 12, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 150})
C = {"gt": "#222222", "pg": "#1f6feb", "naive": "#d1242f", "still": "#8c959f"}
seq = sys.argv[1] if len(sys.argv) > 1 else "V2_01_easy"; d = f"data/dev/{seq}/missing"
meta = json.load(open(f"{d}/meta.json"))
subprocess.run(["build/pathguard", d, "/tmp/pg.txt"], check=True)
tg, pg, _ = load_tum(f"{d}/groundtruth.txt"); te, pe, _ = load_tum("/tmp/pg.txt")
ie, ig = associate(te, tg); te, pe, gp = te[ie], pe[ie], pg[ig]
pre = te < meta["fail_time"]; R, t = umeyama(pe[pre], gp[pre]); pa = pe @ R.T + t
a = load_imu(d); tt, aa, good = times(a, d); pn = naive(aa, good, tt)
i2, j2 = associate(tt, tg); tn, pn, gn = tt[i2], pn[i2], pg[j2]
Rn, tn_ = umeyama(pn[tn < meta["fail_time"]], gn[tn < meta["fail_time"]]); pn = pn @ Rn.T + tn_
T0 = tg[0]
# 1. error over time (log)
fig, ax = plt.subplots(figsize=(9, 4.2))
ax.semilogy(tn - T0, np.linalg.norm(pn - gn, axis=1) + 1e-3, color=C["naive"], label="Naive double integration")
ax.semilogy(te - T0, np.linalg.norm(gp - gp[pre].mean(0), axis=1), color=C["still"], label="Stand still")
ax.semilogy(te - T0, np.linalg.norm(pa - gp, axis=1), color=C["pg"], lw=2, label="PathGuard")
for s, e in meta["dropouts"]: ax.axvspan(s - T0, e - T0, color="#f2cc60", alpha=.35, lw=0)
ax.axvspan(meta["fail_time"] - T0, meta["end_time"] - T0, color="#d1242f", alpha=.12, lw=0)
ax.set_xlabel("time [s]"); ax.set_ylabel("position error [m] (log)"); ax.legend(frameon=False, loc="upper left")
ax.set_title(f"{seq}: yellow = 2 s dropouts, red = IMU dead (blind 5 s)", fontsize=11)
fig.tight_layout(); fig.savefig("figures/error_over_time.png"); plt.close(fig)
# 2. top view
fig, ax = plt.subplots(figsize=(6.5, 6))
ax.plot(gp[:, 0], gp[:, 1], color=C["gt"], lw=1.2, label="Ground truth")
ax.plot(pa[pre, 0], pa[pre, 1], color=C["pg"], lw=1.2, label="PathGuard")
ax.plot(pa[~pre, 0], pa[~pre, 1], color="#d1242f", lw=3, label="Blind 5 s forecast")
ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(frameon=False)
ax.set_title(f"{seq} (top view, unseen flight)", fontsize=11)
fig.tight_layout(); fig.savefig("figures/top_view.png"); plt.close(fig)
print("ok")
