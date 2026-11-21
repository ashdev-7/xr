"""Position error (after evo SE(3) alignment) in time windows relative to the failure. usage: errwin.py <tag> <flights,> <scens,>"""
import sys, json, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score as S
WIN = {"first half": None, "F-30..F-15": (-30, -15), "F-15..F-5": (-15, -5), "F-5..F-1": (-5, -1), "F-1..F": (-1, 0),
       "blind 0-1 s": (0, 1), "blind 1-5 s": (1, 5.1)}


def errwin(tag, flights, scens):
    acc = {}
    for fl in flights:
        for sc in scens:
            f = f"results/{tag}/{fl}__{sc}.txt"; rd = f"data/bench/{fl}/{sc}"
            if not os.path.exists(f):
                continue
            meta = json.load(open(rd + "/meta.json")); ts, v, _ = S.contract(f, rd + "/imu0/data.csv")
            tg, pg, qg = S.read_tum(f"data/bench/{fl}/gt.tum"); t = ts * 1e-9
            pgi, qgi, ok = S.slerp_gt(tg, pg, qg, t); t, pe, pgi = t[ok], v[ok, :3], pgi[ok]
            R, tr = S.umeyama(pe, pgi); e = np.linalg.norm(pe @ R.T + tr - pgi, axis=1)
            F = meta["fail_time"]
            for k, w in WIN.items():
                m = (t < t[0] + (F - t[0]) / 2) if w is None else ((t >= F + w[0]) & (t < F + w[1]))
                acc.setdefault(k, []).append(np.sqrt(np.mean(e[m] ** 2)))
    return {k: float(np.mean(v)) for k, v in acc.items()}


if __name__ == "__main__":
    for k, v in errwin(sys.argv[1], sys.argv[2].split(","), sys.argv[3].split(",")).items():
        print(f"  {k:12s} {v:.3f} m")
