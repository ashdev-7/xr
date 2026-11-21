"""evo-style scorer for Track 3 practice runs.

ATE: SE(3) Umeyama alignment (evo_ape -a) fitted on the IMU-covered part only,
then position RMSE. Reported separately for:
  all      every IMU timestamp (before failure)
  wp       checkpoints every 10 s (our stand-in for "mission waypoints")
  drop     the 2 s dropout windows plus 2 s after each
  blind    the 5 s after IMU failure (same alignment, so it is pure forecast)
RPE: translation error over 1 s windows (evo_rpe --delta 1 --delta_unit s).
"""
import json, os, sys
import numpy as np


def load_tum(path):
    a = np.loadtxt(path, comments="#")
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def associate(t_est, t_gt, tol=0.003):
    j = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    j = np.where(np.abs(t_gt[j - 1] - t_est) < np.abs(t_gt[j] - t_est), j - 1, j)
    ok = np.abs(t_gt[j] - t_est) < tol
    return np.nonzero(ok)[0], j[ok]


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def score(est_path, run_dir):
    meta = json.load(open(os.path.join(run_dir, "meta.json")))
    tg, pg, _ = load_tum(os.path.join(run_dir, "groundtruth.txt"))
    te, pe, _ = load_tum(est_path)
    ie, ig = associate(te, tg)
    te, pe, tgm, pgm = te[ie], pe[ie], tg[ig], pg[ig]
    fail = meta["fail_time"]
    pre = te < fail - 1e-6
    R, tr = umeyama(pe[pre], pgm[pre])
    err = np.linalg.norm(pe @ R.T + tr - pgm, axis=1)
    rms = lambda m: float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else float("nan")
    wp_t = np.arange(te[0] + 10, fail, 10.0)
    wp = np.zeros(len(te), bool)
    wp[np.clip(np.searchsorted(te, wp_t), 0, len(te) - 1)] = True
    dm = np.zeros(len(te), bool)
    for s, e in meta["dropouts"]:
        dm |= (te >= s) & (te <= e + 2.0)
    blind = te >= fail - 1e-6
    # RPE over 1 s (translation), on aligned positions, pre-failure
    k = np.searchsorted(te, te + 1.0)
    okk = (k < len(te)) & pre
    k = k[okk]
    d_est = (pe[k] - pe[okk]) @ R.T
    d_gt = pgm[k] - pgm[okk]
    rpe = float(np.sqrt(np.mean(np.sum((d_est - d_gt) ** 2, 1))))
    n_expected = int(np.sum(tg <= meta["end_time"]))
    return dict(ate=rms(pre), wp=rms(wp), drop=rms(dm), blind=rms(blind),
                blind_end=float(err[blind][-1]) if blind.any() else float("nan"),
                rpe1s=rpe, coverage=len(te) / max(n_expected, 1))


if __name__ == "__main__":
    print(json.dumps(score(sys.argv[1], sys.argv[2]), indent=1))
