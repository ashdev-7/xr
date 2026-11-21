"""Attitude error decomposition vs ground truth: tilt (roll/pitch) and heading, over time."""
import sys, numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0] if "/" in __file__ else ".")
from score import read_tum, read_est, slerp_gt, q2R, umeyama
def decompose(est, gtp):
    ts, v, _ = read_est(est); t = ts * 1e-9
    tg, pg, qg = read_tum(gtp); _, q, ok = slerp_gt(tg, pg, qg, t)
    t, Re, Rg = t[ok], q2R(v[ok, 3:7]), q2R(q[ok])
    # best constant world rotation (heading + small tilt) aligning est to gt attitude
    M = np.einsum("nij,nkj->ik", Rg, Re); U, _, Vt = np.linalg.svd(M); W = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    E = np.einsum("nji,njk->nik", Rg, np.einsum("ij,njk->nik", W, Re))  # body-frame error
    Ew = np.einsum("nij,njk,nlk->nil", Rg, E, Rg)                       # world-frame error
    tilt = np.degrees(np.arccos(np.clip(Ew[:, 2, 2], -1, 1)))
    yaw = np.degrees(np.arctan2(Ew[:, 1, 0], Ew[:, 0, 0]))
    return t - t[0], tilt, yaw
if __name__ == "__main__":
    t, tilt, yaw = decompose(sys.argv[1], sys.argv[2])
    for q in np.linspace(0, len(t) - 1, 12).astype(int):
        print(f"t={t[q]:7.1f}s  tilt={tilt[q]:6.2f}  heading={yaw[q]:7.2f} deg")
    print(f"RMS tilt {np.sqrt(np.mean(tilt**2)):.2f} deg, RMS heading {np.sqrt(np.mean(yaw**2)):.2f} deg")
