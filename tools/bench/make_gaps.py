"""Add three 2-second signal losses (rows removed, as the FAQ defines dropouts) to existing official-degradation
variants, mirroring the deck's "2-second signal loss". Gaps are placed at 30/50/70 % of the flight, outside the
protected start and the failure window. Writes <scen>_g2 variants with updated meta (gap list for the scorer).
usage: python tools/bench/make_gaps.py <scen> [flights...]"""
import json, os, sys
BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "bench")
scen = sys.argv[1]
for fl in sys.argv[2:] or sorted(os.listdir(BENCH)):
    src = os.path.join(BENCH, fl, scen)
    if not os.path.isdir(src):
        continue
    lines = open(os.path.join(src, "imu0", "data.csv")).read().splitlines()
    hdr, rows = lines[0], lines[1:]
    ts = [int(r.split(",")[0]) for r in rows]
    t0, t1 = ts[0], ts[-1]
    cut = set()
    gaps = []
    for frac in (0.3, 0.5, 0.7):
        a = t0 + int(frac * (t1 - t0)); b = a + 2 * 10**9
        idx = [i for i, t in enumerate(ts) if a <= t < b]
        cut.update(idx)
        gaps.append([ts[idx[0] - 1] * 1e-9, ts[idx[-1] + 1] * 1e-9, len(idx)])
    out = [r for i, r in enumerate(rows) if i not in cut]
    base, seed = scen.rsplit("_s", 1)
    dst = os.path.join(BENCH, fl, f"{base}_g2_s{seed}")
    os.makedirs(os.path.join(dst, "imu0"), exist_ok=True)
    open(os.path.join(dst, "imu0", "data.csv"), "w").write("\n".join([hdr] + out) + "\n")
    meta = json.load(open(os.path.join(src, "meta.json")))
    meta["gaps"] = meta["gaps"] + gaps; meta["rows"] = len(out)
    json.dump(meta, open(os.path.join(dst, "meta.json"), "w"), indent=1)
    print(fl, "removed", len(cut), "rows in 3 x 2 s gaps")
