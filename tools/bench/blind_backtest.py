"""Blind-window backtest: choose the position/attitude forecast on many realistic failure windows.

Each TRAIN flight is cut at 45/60/75/90 % of its raw length and the OFFICIAL degradation is re-applied to the
shortened log (make_bench.py --trunc), so every cut ends with the real failure at maximum jamming. This gives
8 blind windows per flight instead of 1. usage: python tools/bench/blind_backtest.py
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run import run  # noqa: E402

SC = tuple(f"{s}_T{t}" for s in ("aware", "asis") for t in ("45", "60", "75", "90"))
CLASSES = (("drone", ["MH_01_easy", "MH_03_medium", "MH_05_difficult", "V1_01_easy", "V2_02_medium"]),
           ("handheld", ["TUMVI_room1", "TUMVI_room3", "TUMVI_room5"]))

if __name__ == "__main__":
    for cls, fl in CLASSES:
        print(f"== {cls}: {len(fl)} TRAIN flights x 8 backtest cuts", flush=True)
        for bt in (0.0, 0.3, 0.7, -1.0):
            line = []
            for at in (0.0, 0.3, 1.0):
                _, res = run("pg", [f"--blind_tau={bt}", f"--att_tau={at}"], fl, SC, (42,), tag="bt", quiet=True)
                b = np.mean([r["blind"] for r in res])
                a = np.mean([r["are_blind"] for r in res if np.isfinite(r["are_blind"])])
                line.append(f"att_tau={at}: blind {b:.3f} m, att {a:5.1f} deg")
            print(f"  blind_tau={bt:4}: " + " | ".join(line) + f"  (n={len(res)})", flush=True)
