"""Error-budget oracles (development only). Imports src/main.py as a module and swaps ONE stage
for ground truth, so main.py itself never sees ground truth.

usage: python tools/bench/oracle.py <oracle> <variant dir> <out.txt> [--param=value ...]
oracles
  none      the normal pipeline (sanity: must equal main.py output)
  gtatt     translation stage fed with ground-truth attitude (removes all attitude error)
  gtatt_tilt ground-truth roll/pitch, estimated heading (removes tilt error only)
"""
import os, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
sys.path.insert(0, os.path.dirname(__file__))
import main as M  # noqa: E402
from score import read_tum, slerp_gt  # noqa: E402
from array import array  # noqa: E402


def yaw_of(qw, qx, qy, qz):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def qz_rot(a):
    return np.stack([np.cos(a / 2), 0 * a, 0 * a, np.sin(a / 2)], 1)


def qmul_np(a, b):
    aw, ax, ay, az = a.T; bw, bx, by, bz = b.T
    return np.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], 1)


def run(oracle, vdir, out, params):
    for a in params:
        k, v = a[2:].split("=", 1); M.P[k] = type(M.P[k])(float(v))
    opener, src = M.resolve_input(vdir)
    info = M.survey(opener); M.INFO = info
    sg, sa, T_L, n_valid = M.noise_meter(opener, info)
    ramp_g, ramp_a = M.fit_ramp(sg, T_L), M.fit_ramp(sa, T_L)
    gsum, asum, cnt, nb, st = M.preintegrate(opener, info, sg, sa, T_L)
    M.INIT = M.initial_attitude(gsum, asum, cnt, nb, sg)
    qs, _ = M.attitude(gsum, asum, cnt, nb, sg, sa, ramp_g, ramp_a, T_L, M.INIT[2])
    if oracle.startswith("gtatt"):
        tg, pg, qg = read_tum(os.path.join(os.path.dirname(vdir.rstrip("/\\")), "gt.tum"))
        t0 = info["first_ts"] * 1e-9
        tb = t0 + np.arange(nb + 1) * M.P["bin_dt"]
        _, q, ok = slerp_gt(tg, pg, qg, tb, max_gap=0.2)
        # outside GT coverage hold the nearest GT attitude
        idx = np.nonzero(ok)[0]
        q[:idx[0]] = q[idx[0]]; q[idx[-1] + 1:] = q[idx[-1]]
        qg_wxyz = np.stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]], 1)
        if oracle == "gtatt_tilt":  # keep estimated heading, take GT tilt
            qe = np.array(qs).reshape(-1, 4)
            dy = yaw_of(*qe.T) - yaw_of(*qg_wxyz.T)
            qg_wxyz = qmul_np(qz_rot(dy), qg_wxyz)
        qs = array("f", qg_wxyz.astype(np.float32).ravel().tolist())
    pos, vel, K, kdt, gz = M.translation(asum, cnt, nb, qs, T_L, max(info["t_end"], T_L), ramp_a)
    M.write_output(opener, info, out, qs, nb, pos, vel, K, kdt, T_L)


if __name__ == "__main__":
    run(sys.argv[1], sys.argv[2], sys.argv[3], [a for a in sys.argv[4:] if a.startswith("--")])
