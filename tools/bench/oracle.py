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

    def hook(stage, d):
        if stage != "attitude" or not oracle.startswith("gtatt"):
            return d["qs"]
        qs, nb, info = d["qs"], d["nb"], d["info"]
        tg, pg, qg = read_tum(os.path.join(os.path.dirname(vdir.rstrip("/\\")), "gt.tum"))
        t0 = info["first_ts"] * 1e-9
        tb = t0 + np.arange(nb + 1) * M.P["bin_dt"]
        _, q, ok = slerp_gt(tg, pg, qg, tb, max_gap=0.2)
        idx = np.nonzero(ok)[0]
        q[:idx[0]] = q[idx[0]]; q[idx[-1] + 1:] = q[idx[-1]]
        qg_wxyz = np.stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]], 1)
        if oracle == "gtatt_tilt":  # keep estimated heading, take GT tilt
            qe = np.array(qs).reshape(-1, 4)
            dy = yaw_of(*qe.T) - yaw_of(*qg_wxyz.T)
            qg_wxyz = qmul_np(qz_rot(dy), qg_wxyz)
        return array("f", qg_wxyz.astype(np.float32).ravel().tolist())

    M.run(vdir, out, hook=hook)


if __name__ == "__main__":
    run(sys.argv[1], sys.argv[2], sys.argv[3], [a for a in sys.argv[4:] if a.startswith("--")])
