"""Figures for README / deck, computed from benchmark data and the report run (results/rep_pg, rep_still).

usage: python tools/figures.py      (after tools/bench/report.py)
writes figures/*.png
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "bench"))
sys.path.insert(0, HERE)
from score import read_est, read_tum, slerp_gt, umeyama, q2R  # noqa: E402

OUT = os.path.join(ROOT, "figures")
DARK = "--dark" in sys.argv  # deck version: colours of the organizers' dark template
BENCH = os.path.join(ROOT, "data", "bench")
if DARK:
    C_GT, C_PG, C_ST, C_BAD = "#E6EDF3", "#29B6F6", "#8B95A3", "#FF6B6B"
    BG, FG = "#0B0F1A", "#C9D1D9"
    OUT = os.path.join(OUT, "dark")
    plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG, "text.color": FG,
                         "axes.labelcolor": FG, "axes.edgecolor": "#5A6472", "xtick.color": FG, "ytick.color": FG,
                         "legend.labelcolor": FG})
else:
    C_GT, C_PG, C_ST, C_BAD = "#222222", "#1f6fd1", "#9aa0a6", "#d93025"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 150})


def aligned(est_path, flight):
    ts, v, _ = read_est(est_path); t = ts * 1e-9
    tg, pg, qg = read_tum(os.path.join(BENCH, flight, "gt.tum"))
    p, q, ok = slerp_gt(tg, pg, qg, t)
    R, tr = umeyama(v[ok, :3], p[ok])
    return t[ok], v[ok, :3] @ R.T + tr, p[ok]


def fig_degradation(flight="MH_01_easy", scen="aware_s42"):
    a = np.loadtxt(os.path.join(BENCH, flight, scen, "imu0", "data.csv"), delimiter=",", comments="#")
    t = (a[:, 0] - a[0, 0]) * 1e-9
    f = np.linalg.norm(a[:, 4:7], axis=1)
    fig, ax = plt.subplots(2, 1, figsize=(8, 3.6), sharex=True, gridspec_kw=dict(height_ratios=[2, 1]))
    ax[0].plot(t, f, lw=0.4, color=C_PG)
    dead = f == 0
    ax[0].fill_between(t, 0, 1, where=dead, transform=ax[0].get_xaxis_transform(), color=C_BAD, alpha=0.15, lw=0)
    ax[0].text(t[dead][0] if dead.any() else t[-1], 0.92, "IMU failure: 5 s of zeros ", transform=ax[0].get_xaxis_transform(),
               color=C_BAD, fontsize=8, va="top", ha="right")
    ax[0].set_ylabel("|specific force| [m/s²]")
    ax[0].set_title(f"Official degradation on real EuRoC {flight.split('_')[0]}_{flight.split('_')[1]}: rising noise, drift, spikes, dropouts, failure", fontsize=9)
    d = np.diff(t)
    gaps = np.nonzero(d > 0.0075)[0]
    sec = t[gaps].astype(int)
    cnt = np.bincount(sec, weights=np.round(d[gaps] / 0.005) - 1, minlength=int(t[-1]) + 1)
    ax[1].bar(np.arange(len(cnt)), cnt, width=1.0, color=C_ST)
    ax[1].set_ylabel("missing rows / s"); ax[1].set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "degradation.png")); plt.close(fig)


def fig_topview(flights=("MH_02_easy", "V1_02_medium", "TUMVI_room2"), scen="aware_s42"):
    fig, axs = plt.subplots(1, len(flights), figsize=(9, 3.2))
    for ax, fl in zip(axs, flights):
        pg_ = os.path.join(ROOT, "results", "rep_pg", f"{fl}__{scen}.txt")
        st_ = os.path.join(ROOT, "results", "rep_still", f"{fl}__{scen}.txt")
        if not os.path.exists(pg_):
            continue
        t, pe, p = aligned(pg_, fl)
        ax.plot(p[:, 0], p[:, 1], color=C_GT, lw=1.0, label="ground truth")
        ax.plot(pe[:, 0], pe[:, 1], color=C_PG, lw=1.0, label="PathGuard")
        if os.path.exists(st_):
            _, ps, _ = aligned(st_, fl)
            ax.plot(ps[:1, 0], ps[:1, 1], "o", color=C_ST, ms=6, label="stand-still")
        e = np.sqrt(np.mean(np.sum((pe - p) ** 2, 1)))
        ax.set_title(f"{fl.replace('_', ' ')}  ATE {e:.2f} m", fontsize=9)
        ax.set_aspect("equal", "datalim"); ax.set_xlabel("x [m]")
    axs[0].set_ylabel("y [m]"); axs[0].legend(fontsize=7, frameon=False, loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "top_view.png")); plt.close(fig)


def fig_drag(flight="MH_02_easy", t_air=40):
    a = np.loadtxt(os.path.join(ROOT, "data", "raw", "euroc", flight, "imu0.csv"), delimiter=",", comments="#")
    g = np.loadtxt(os.path.join(ROOT, "data", "raw", "euroc", flight, "gt.csv"), delimiter=",", comments="#")
    t = a[:, 0] * 1e-9; f = a[:, 4:7]; tg = g[:, 0] * 1e-9
    tgum, pg, qg = read_tum(os.path.join(BENCH, flight, "gt.tum")); _, q, ok = slerp_gt(tgum, pg, qg, t)
    vw = np.stack([np.interp(t, tg, g[:, 8 + k]) for k in range(3)], 1); vb = np.einsum("nji,nj->ni", q2R(q), vw)
    k = np.ones(400) / 400
    lp = lambda x: np.stack([np.convolve(x[:, i], k, "same") for i in range(3)], 1)
    fl_, vbl = lp(f), lp(vb)
    air = ok & (t > t[0] + t_air) & (t < t[-1] - 8)
    u = fl_[air].mean(0); u /= np.linalg.norm(u); e1 = np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1)
    x, y = vbl[air] @ e1, fl_[air] @ e1
    x, y = x - x.mean(), y - y.mean()
    mu = -np.sum(x * y) / np.sum(x * x); r2 = np.corrcoef(x, y)[0, 1] ** 2
    fig, ax = plt.subplots(1, 2, figsize=(8, 2.8), gridspec_kw=dict(width_ratios=[2, 1]))
    tt = t[air] - t[air][0]
    ax[0].plot(tt, x, color=C_GT, lw=1, label="true body velocity (GT)")
    ax[0].plot(tt, -y / mu, color=C_PG, lw=1, label="−f⊥ / μ from the accelerometer")
    ax[0].set_xlabel("time airborne [s]"); ax[0].set_ylabel("velocity [m/s]"); ax[0].legend(fontsize=7, frameon=False)
    ax[1].plot(x[::50], y[::50], ".", ms=2, color=C_PG)
    ax[1].set_xlabel("v⊥ [m/s]"); ax[1].set_ylabel("f⊥ [m/s²]")
    ax[1].set_title(f"rotor drag: μ = {mu:.2f} 1/s, R² = {r2:.2f}", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "rotor_drag.png")); plt.close(fig)


def fig_error_time(flight="MH_02_easy", scen="aware_s42"):
    fig, ax = plt.subplots(figsize=(8, 2.6))
    t = None
    for tag, name, col in (("rep_still", "stand-still", C_ST), ("rep_pg", "PathGuard", C_PG)):
        pth = os.path.join(ROOT, "results", tag, f"{flight}__{scen}.txt")
        if not os.path.exists(pth):
            continue
        t, pe, p = aligned(pth, flight)
        ax.plot(t - t[0], np.linalg.norm(pe - p, axis=1), color=col, lw=1, label=name)
    if t is None:
        raise RuntimeError("run tools/bench/report.py first")
    ax.axvspan(t[-1] - t[0] - 5, t[-1] - t[0], color=C_BAD, alpha=0.1, lw=0)
    ax.set_xlabel("time [s]"); ax.set_ylabel("position error [m]"); ax.legend(fontsize=8, frameon=False)
    ax.set_title(f"{flight.replace('_', ' ')} ({scen.split('_')[0]}): error over time; shaded = blind 5 s", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "error_over_time.png")); plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for fn in (fig_degradation, fig_topview, fig_drag, fig_error_time):
        try:
            fn(); print("ok", fn.__name__)
        except Exception as e:  # keep going; report which figure failed
            print("FAILED", fn.__name__, e)
