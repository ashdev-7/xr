"""Run the estimator on every practice flight and print a score table.

usage: python tools/bench.py [--bin build/pathguard] [--split train|test|all] [--param=value ...]
Train/test split is by flight, so tuned parameters are always reported on unseen flights.
"""
import json, os, subprocess, sys, tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from score import score

TRAIN = ["MH_01_easy", "MH_03_medium", "MH_05_difficult", "V1_01_easy", "V1_03_difficult", "V2_02_medium"]
TEST = ["MH_02_easy", "MH_04_difficult", "V1_02_medium", "V2_01_easy", "V2_03_difficult"]


def run(binary, params, split="all", styles=("missing", "zeros"), quiet=False):
    seqs = {"train": TRAIN, "test": TEST, "all": TRAIN + TEST}[split]
    rows = []
    with tempfile.TemporaryDirectory() as td:
        for seq in seqs:
            for st in styles:
                d = os.path.join("data/dev", seq, st)
                out = os.path.join(td, f"{seq}_{st}.txt")
                subprocess.run([binary, d, out, *params], check=True)
                s = score(out, d); s["seq"] = f"{seq}/{st}"; rows.append(s)
    keys = ["ate", "wp", "drop", "blind", "blind_end", "rpe1s"]
    if not quiet:
        print(f"{'flight':28s}" + "".join(f"{k:>10s}" for k in keys) + f"{'rows':>7s}")
        for r in rows:
            print(f"{r['seq']:28s}" + "".join(f"{r[k]:10.3f}" for k in keys) + f"{r['coverage']:7.3f}")
    mean = {k: float(np.mean([r[k] for r in rows])) for k in keys}
    print(f"{'MEAN ' + split:28s}" + "".join(f"{mean[k]:10.3f}" for k in keys))
    return mean, rows


if __name__ == "__main__":
    binary, split, params = "build/pathguard", "all", []
    a = sys.argv[1:]
    while a:
        x = a.pop(0)
        if x == "--bin": binary = a.pop(0)
        elif x == "--split": split = a.pop(0)
        else: params.append(x)
    run(binary, params, split)
