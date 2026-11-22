"""Vertical-motion fidelity of a run: z std (est / gt), correlation, evo alignment tilt. usage: zstats.py <tag>"""
import sys, glob, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score as S
tag = sys.argv[1]; acc = []
for f in sorted(glob.glob(f"results/{tag}/*.txt")):
    if "snapshot" in f: continue
    fl, sc = os.path.basename(f)[:-4].split("__")
    ts, v, _ = S.read_est(f); tg, pg, qg = S.read_tum(f"data/bench/{fl}/gt.tum"); pgi, _, ok = S.slerp_gt(tg, pg, qg, ts * 1e-9)
    pe, pgi = v[ok, :3], pgi[ok]
    ze, zg = pe[:, 2] - pe[:, 2].mean(), pgi[:, 2] - pgi[:, 2].mean()
    R, _ = S.umeyama(pe, pgi)
    acc.append((ze.std(), zg.std(), np.corrcoef(ze, zg)[0, 1], np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))))
a = np.array(acc); print(f"z std est {a[:,0].mean():.3f} gt {a[:,1].mean():.3f} corr {a[:,2].mean():.2f} | alignment tilt median {np.median(a[:,3]):.1f} deg (n={len(a)})")
