"""Choose the blind-window shrink factor on TRAIN backtests (4 cuts x 2 column scenarios per flight).
usage: python tools/bench/blind_shrink_sweep.py"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run import run  # noqa: E402
SC = tuple(f"{s}_T{t}" for s in ("aware", "asis") for t in ("45", "60", "75", "90"))
CLASSES = (("drone", ["MH_01_easy", "MH_03_medium", "MH_05_difficult", "V1_01_easy", "V2_02_medium"]),
           ("handheld", ["TUMVI_room1", "TUMVI_room3", "TUMVI_room5"]))
if __name__ == "__main__":
    for cls, fl in CLASSES:
        _, st = run("still", [], fl, SC, (42,), tag="bs_still", quiet=True)
        print(f"== {cls}: stand-still blind {np.mean([r['blind'] for r in st]):.3f}", flush=True)
        for shr in (1.0, 0.8, 0.6, 0.4, 0.2):
            for ramp in ((1.0,) if shr == 1.0 else (0.0, 1.0, 3.0)):
                _, res = run("pg", [f"--blind_shrink={shr}", f"--blind_ramp={ramp}"], fl, SC, (42,), tag="bs", quiet=True)
                print(f"  shrink {shr} ramp {ramp}: blind {np.mean([r['blind'] for r in res]):.3f}  "
                      f"ate {np.mean([r['ate'] for r in res]):.3f}  (n={len(res)})", flush=True)
