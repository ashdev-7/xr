"""Scorer for Track 03 (evo-faithful; cross-checked against evo_ape / evo_rpe in check_evo.py).

Contract checks (fatal in the official pipeline):
  * exactly one pose per input row, same order, timestamp == input ns * 1e-9 (compared exactly, as integers)
  * 8 columns, finite numbers, unit quaternions

Metrics (positions after SE(3) Umeyama alignment fitted on ALL matched poses, as the FAQ states):
  ate      RMSE over every row with ground truth (incl. the blind window)
  wp       RMSE over the pre-failure rows   (T1 checkpoints; the hidden waypoints are a subset of these)
  wp10     RMSE at checkpoints every 10 s  (a literal reading of "waypoints"; noisier)
  blind    RMSE over the rows in the 5 s after IMU failure (T2)
  drop     RMSE over rows from each burst dropout (>=5 missing rows) to 1 s after it (T3)
  rpe_t    translation RPE over 1 s (m), evo_rpe --delta 1 --delta_unit s style
  rpe_r    rotation RPE over 1 s (deg)
  are      absolute rotation error after alignment (deg, RMSE)
"""
import json, os, sys
import numpy as np


# ----------------------------------------------------------------------------- io
def read_input_ns(csv_path):
    ts = []
    with open(csv_path) as f:
        for line in f:
            s = line.strip()
            if not s or not (s[0].isdigit()):
                continue
            ts.append(int(s.split(",")[0]))
    return np.array(ts, dtype=np.int64)


def parse_ts_ns(tok):
    """Exact seconds-string -> integer ns (no float rounding)."""
    if "e" in tok or "E" in tok:
        return int(round(float(tok) * 1e9))
    neg = tok.startswith("-")
    if "." in tok:
        a, b = tok.lstrip("-").split(".")
        b = (b + "000000000")[:9]
        v = int(a or "0") * 10**9 + int(b)
    else:
        v = int(tok.lstrip("-")) * 10**9
    return -v if neg else v


def read_est(path):
    ts, vals, bad = [], [], []
    with open(path) as f:
        for ln, line in enumerate(f, 1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split()
            if len(p) != 8:
                bad.append((ln, f"{len(p)} columns")); continue
            ts.append(parse_ts_ns(p[0])); vals.append([float(x) for x in p[1:]])
    return np.array(ts, dtype=np.int64), np.array(vals, dtype=np.float64).reshape(-1, 7), bad


def read_tum(path):
    a = np.loadtxt(path, comments="#")
    return a[:, 0], a[:, 1:4], a[:, 4:8]  # t, p, q(xyzw)


# ----------------------------------------------------------------------------- geometry
def q2R(q):  # xyzw
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))


def slerp_gt(tg, pg, qg, t, max_gap=0.05):
    """Interpolate ground truth at times t; returns mask of usable samples."""
    j = np.searchsorted(tg, t)
    ok = (j > 0) & (j < len(tg))
    j = np.clip(j, 1, len(tg) - 1)
    t0, t1 = tg[j - 1], tg[j]
    ok &= (t1 - t0) < max_gap
    s = np.clip((t - t0) / np.maximum(t1 - t0, 1e-12), 0, 1)[:, None]
    p = pg[j - 1] * (1 - s) + pg[j] * s
    q0, q1 = qg[j - 1].copy(), qg[j].copy()
    d = np.sum(q0 * q1, 1); q1[d < 0] *= -1; d = np.abs(d)
    th = np.arccos(np.clip(d, -1, 1))[:, None]
    sn = np.sin(th)
    w0 = np.where(sn > 1e-9, np.sin((1 - s) * th) / np.where(sn > 1e-9, sn, 1), 1 - s)
    w1 = np.where(sn > 1e-9, np.sin(s * th) / np.where(sn > 1e-9, sn, 1), s)
    q = q0 * w0 + q1 * w1
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return p, q, ok


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md) / len(src)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


# ----------------------------------------------------------------------------- scoring
def contract(est_path, input_csv):
    ts_in = read_input_ns(input_csv)
    ts, v, bad = read_est(est_path)
    issues = []
    if bad:
        issues.append(f"{len(bad)} malformed lines, first {bad[0]}")
    if len(ts) != len(ts_in):
        issues.append(f"row count {len(ts)} != input {len(ts_in)}")
    else:
        mism = np.nonzero(ts != ts_in)[0]
        if len(mism):
            issues.append(f"{len(mism)} timestamp mismatches, first row {mism[0]}: {ts[mism[0]]} vs {ts_in[mism[0]]}")
    if len(v) and not np.isfinite(v).all():
        issues.append("non-finite values")
    if len(v):
        qn = np.linalg.norm(v[:, 3:7], axis=1)
        if np.abs(qn - 1).max() > 1e-4:
            issues.append(f"non-unit quaternion, max |norm-1| = {np.abs(qn-1).max():.2e}")
    return ts, v, issues


def score(est_path, run_dir, gt=None):
    meta = json.load(open(os.path.join(run_dir, "meta.json")))
    ts, v, issues = contract(est_path, os.path.join(run_dir, "imu0", "data.csv"))
    tg, pg, qg = gt if gt is not None else read_tum(os.path.join(os.path.dirname(run_dir.rstrip("/\\")), "gt.tum"))
    t = ts * 1e-9
    pe, qe = v[:, :3], v[:, 3:7]
    pgi, qgi, ok = slerp_gt(tg, pg, qg, t)
    t, pe, qe, pgi, qgi = t[ok], pe[ok], qe[ok], pgi[ok], qgi[ok]
    R, tr = umeyama(pe, pgi)
    pa = pe @ R.T + tr
    err = np.linalg.norm(pa - pgi, axis=1)
    rms = lambda m: float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else float("nan")
    fail = meta["fail_time"]
    pre, blind = t < fail - 1e-9, t >= fail - 1e-9
    wp_t = np.arange(t[0] + 10.0, fail, 10.0)
    wpi = np.clip(np.searchsorted(t, wp_t), 0, len(t) - 1)
    wp10 = float(np.sqrt(np.mean(err[wpi] ** 2))) if len(wpi) else float("nan")
    dm = np.zeros(len(t), bool)
    for a, b, nmiss in meta["gaps"]:
        if nmiss >= 5:
            dm |= (t >= a) & (t <= b + 1.0)
    # rotation: aligned estimate R*Re vs Rg
    Re, Rg = q2R(qe), q2R(qgi)
    Rea = np.einsum("ij,njk->nik", R, Re)
    are = rot_angle(np.einsum("nji,njk->nik", Rg, Rea))
    # RPE over 1 s, pre-failure starts (evo: relative SE3 error between pose pairs)
    k = np.searchsorted(t, t + 1.0)
    m = (k < len(t)) & pre
    i, k = np.nonzero(m)[0], k[m]
    dRe = np.einsum("nji,njk->nik", Rea[i], Rea[k])
    dRg = np.einsum("nji,njk->nik", Rg[i], Rg[k])
    dpe = np.einsum("nji,nj->ni", Rea[i], pa[k] - pa[i])
    dpg = np.einsum("nji,nj->ni", Rg[i], pgi[k] - pgi[i])
    # E = inv(dG) dE  -> translation part = dRg^T (dpe - dpg)
    et = np.einsum("nji,nj->ni", dRg, dpe - dpg)
    er = rot_angle(np.einsum("nji,njk->nik", dRg, dRe))
    return dict(ate=rms(np.ones(len(t), bool)), wp=rms(pre), wp10=wp10, blind=rms(blind), drop=rms(dm),
                rpe_t=float(np.sqrt(np.mean(np.sum(et ** 2, 1)))), rpe_r=float(np.sqrt(np.mean(er ** 2))),
                are=float(np.sqrt(np.mean(are ** 2))), blind_end=float(err[blind][-1]) if blind.any() else float("nan"),
                n=int(ok.sum()), issues=issues)


if __name__ == "__main__":
    r = score(sys.argv[1], sys.argv[2])
    print(json.dumps(r, indent=1))
