"""Flight-level statistics (the flight is the experimental unit; seeds and column scenarios are averaged within a
flight first). For each metric: mean PathGuard - stand-still difference with a bootstrap 95 % CI over flights,
the number of flights where PathGuard is better, and the median ratio.
usage: python tools/bench/stats.py <pg tag> <still tag> [label]"""
import sys, os, glob, random
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score as S

pg_tag, st_tag = sys.argv[1], sys.argv[2]
label = sys.argv[3] if len(sys.argv) > 3 else pg_tag
per = {}
for f in sorted(glob.glob(f"results/{pg_tag}/*.txt")):
    b = os.path.basename(f)
    if "snapshot" in b or not os.path.exists(f"results/{st_tag}/{b}"):
        continue
    fl, sc = b[:-4].split("__"); rd = f"data/bench/{fl}/{sc}"
    a, s = S.score(f, rd), S.score(f"results/{st_tag}/{b}", rd)
    per.setdefault(fl, []).append({k: (a[k], s[k]) for k in ("wp", "blind", "drop", "rpe_t_all", "ate")})
rng = random.Random(0)
fls = sorted(per)
print(f"== {label}: {len(fls)} flights, {sum(len(v) for v in per.values())} runs (flight = unit)")
for k, name in (("wp", "pre-failure (waypoints)"), ("blind", "blind 5 s"), ("drop", "dropout"), ("rpe_t_all", "RPE 1 s"), ("ate", "ATE")):
    d = np.array([np.mean([r[k][0] - r[k][1] for r in per[fl]]) for fl in fls])
    ratio = np.array([np.mean([r[k][0] for r in per[fl]]) / np.mean([r[k][1] for r in per[fl]]) for fl in fls])
    boots = [np.mean([d[rng.randrange(len(d))] for _ in d]) for _ in range(10000)]
    lo, hi = np.percentile(boots, [2.5, 97.5])
    print(f"  {name:24s} PG - still {d.mean():+.3f} m  95% CI [{lo:+.3f}, {hi:+.3f}]  better on {int((d < 0).sum())}/{len(d)} flights"
          f"  median ratio {np.median(ratio):.2f}")
