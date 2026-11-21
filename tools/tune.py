"""Coordinate-descent tuning on TRAIN flights only (objective: mean of ATE, waypoint, dropout, blind errors)."""
import os, sys, json, subprocess, tempfile, itertools
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from score import score
from bench import TRAIN, TEST
BIN = "build/pathguard"
def evaluate(params, seqs):
    jobs = [(s, st) for s in seqs for st in ("missing", "zeros")]
    def one(j):
        s, st = j; d = f"data/dev/{s}/{st}"
        fd, out = tempfile.mkstemp(suffix=".txt"); os.close(fd)
        subprocess.run([BIN, d, out, *[f"--{k}={v}" for k, v in params.items()]], check=True)
        r = score(out, d); os.unlink(out); return r
    with ThreadPoolExecutor(8) as ex: rs = list(ex.map(one, jobs))
    m = {k: float(np.mean([r[k] for r in rs])) for k in ("ate", "wp", "drop", "blind", "rpe1s")}
    m["obj"] = 0.3 * m["ate"] + 0.2 * m["wp"] + 0.2 * m["blind"] + 0.1 * m["drop"] + 0.2 * m["rpe1s"]
    return m
if __name__ == "__main__":
    space = json.loads(sys.argv[1]); params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    best = evaluate(params, TRAIN); print("start", best, flush=True)
    for rnd in range(2):
        for k, vals in space.items():
            for v in vals:
                trial = dict(params, **{k: v}); m = evaluate(trial, TRAIN)
                if m["obj"] < best["obj"] - 1e-4: best, params = m, trial; print(f"  {k}={v} obj={m['obj']:.3f} ate={m['ate']:.2f} blind={m['blind']:.2f}", flush=True)
    print("BEST", json.dumps(params)); print("train", best); print("test ", evaluate(params, TEST))
