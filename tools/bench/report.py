"""Generate every number used in README / deck / video from the CURRENT code (no hand-typed results).

Runs on the TEST flights (never used for tuning), both column scenarios, seeds 42, 1, 2:
  * PathGuard (final), stand-still, naive double integration, legacy C++ v1
  * ablations (one component switched off at a time)
  * resources: serial runtime (ms per s of IMU) and traced heap, per platform class
Writes results/report.json, results/report.md and the README section between
<!-- RESULTS:BEGIN --> and <!-- RESULTS:END -->.
usage: python tools/bench/report.py [--quick]   (--quick: seed 42 only)
"""
import json, os, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from run import run  # noqa: E402
from tune import SPLITS  # noqa: E402

KEYS = ["ate", "wp", "blind", "drop", "rpe_t", "rpe_r"]
ABL = [("full", []), ("no rotor-drag velocity", ["--drag=0"]), ("no rest detection (ZUPT/ZARU)", ["--still=0"]),
       ("no spike repair", ["--repair=0"]), ("no jamming adaptation", ["--adapt=0"]),
       ("no attitude smoother (filter only)", ["--backward=0"]), ("blind: smoother forecast (no hold)", ["--blind_tau=-1"])]


def table(rows, header):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out)


def mean_of(res, flights):
    rs = [r for r in res if r["flight"] in flights]
    return {k: float(np.mean([r[k] for r in rs])) for k in KEYS} if rs else None


def resources(flight):
    d = os.path.join(ROOT, "data", "bench", flight, "aware_s42")
    r = subprocess.run([sys.executable, os.path.join(HERE, "measure.py"), d], capture_output=True, text=True)
    m = json.loads(r.stdout[r.stdout.index("{"):])
    # serial runtime without a profiler (median of 3)
    code = ("import sys,time; sys.path.insert(0,%r); import main as M; t=time.perf_counter(); "
            "M.run(%r, %r); print(time.perf_counter()-t)") % (os.path.join(ROOT, "src"), d, os.path.join(ROOT, "results", "_r.txt"))
    ts = sorted(float(subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True).stdout) for _ in range(3))
    code2 = ("import tracemalloc,sys; sys.path.insert(0,%r); import main as M; tracemalloc.start(); M.run(%r, %r); "
             "print(tracemalloc.get_traced_memory()[1])") % (os.path.join(ROOT, "src"), d, os.path.join(ROOT, "results", "_r.txt"))
    heap = int(subprocess.run([sys.executable, "-I", "-c", code2], capture_output=True, text=True).stdout)
    return dict(flight=flight, duration_s=m["duration_s"], rows=m["rows"], ms_per_s=1000 * ts[1] / m["duration_s"],
                data_heap_mb=heap / 2**20, total_heap_mb=m["peak_heap_mb"])


def main(quick=False):
    seeds = (42,) if quick else (42, 1, 2)
    have = set(os.listdir(os.path.join(ROOT, "data", "bench")))
    test = {c: [f for f in SPLITS[c][1] if f in have] for c in ("rotor", "handheld")}
    allf = test["rotor"] + test["handheld"]
    R = {}
    for name, method, params in [("PathGuard", "pg", []), ("stand-still", "still", []), ("naive integration", "naive", []),
                                 ("legacy C++ v1", "cpp", [])]:
        _, res = run(method, params, allf, ("asis", "aware"), seeds, tag="rep_" + method, quiet=True)
        R[name] = res
    A = {}
    for name, params in ABL:
        _, res = run("pg", params, allf, ("asis", "aware"), seeds, tag="rep_abl", quiet=True) if params else (None, R["PathGuard"])
        A[name] = res
    rsrc = [resources(f) for f in (test["rotor"][:1] + test["handheld"][:1])]
    out = dict(test=test, seeds=seeds, methods={k: v for k, v in R.items()}, ablations=A, resources=rsrc)
    json.dump(out, open(os.path.join(ROOT, "results", "report.json"), "w"), indent=1, default=float)

    md = []
    for cls, title in (("rotor", "Rotorcraft (EuRoC MAV)"), ("handheld", "Handheld (TUM-VI room)")):
        rows = []
        for name, res in R.items():
            m = mean_of(res, test[cls])
            if m:
                rows.append([name] + [f"{m[k]:.3f}" if m[k] < 100 else f"{m[k]:.0f}" for k in KEYS])
        md.append(f"**{title}, TEST flights {', '.join(test[cls])}; 2 scenarios x {len(seeds)} seeds**\n")
        md.append(table(rows, ["method", "ATE [m]", "pre-failure [m]", "blind 5 s [m]", "dropout [m]", "RPE 1 s [m]", "RPE 1 s [deg]"]))
        md.append("")
    rows = []
    for name, res in A.items():
        mr, mh = mean_of(res, test["rotor"]), mean_of(res, test["handheld"])
        rows.append([name] + [f"{m['ate']:.3f} / {m['blind']:.3f}" if m else "-" for m in (mr, mh)])
    md.append("**Ablations (TEST): ATE / blind 5 s [m]**\n")
    md.append(table(rows, ["configuration", "rotorcraft", "handheld"]))
    md.append("")
    rows = [[r["flight"], f"{r['duration_s']:.0f} s ({r['rows']} rows)", f"{r['ms_per_s']:.2f}", f"{r['data_heap_mb']:.2f}",
             f"{r['total_heap_mb']:.2f}"] for r in rsrc]
    md.append("**Resources (single thread, CPython " + sys.version.split()[0] + ")**\n")
    md.append(table(rows, ["flight", "length", "ms per 1 s of IMU (budget 20)", "heap: estimator data [MB]",
                           "heap: incl. interpreter compile of main.py [MB]"]))
    text = "\n".join(md)
    open(os.path.join(ROOT, "results", "report.md"), "w").write(text + "\n")
    readme = os.path.join(ROOT, "README.md")
    s = open(readme, encoding="utf-8").read()
    a, b = "<!-- RESULTS:BEGIN -->", "<!-- RESULTS:END -->"
    if a in s and b in s:
        s = s[:s.index(a) + len(a)] + "\n" + text + "\n" + s[s.index(b):]
        open(readme, "w", encoding="utf-8", newline="\n").write(s)
    print(text)


if __name__ == "__main__":
    main("--quick" in sys.argv)
