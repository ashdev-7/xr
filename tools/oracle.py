"""Oracle floor: ATE if all motion faster than a given period were known perfectly (and nothing slower)."""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from bench import TRAIN, TEST
from score import umeyama
from scipy.signal import butter, filtfilt
seqs = {"train": TRAIN, "test": TEST, "all": TRAIN + TEST}[sys.argv[1] if len(sys.argv) > 1 else "all"]
for T in [2, 5, 10, 20, 40]:
    v = []
    for seq in seqs:
        p = np.loadtxt(f"data/gt/{seq}.csv", delimiter=",", comments="#")[:, 1:4]
        hp = filtfilt(*butter(2, (1 / T) / 100, "high"), p, axis=0)
        R, t = umeyama(hp, p); v.append(np.sqrt(np.mean(np.sum((hp @ R.T + t - p) ** 2, 1))))
    print(f"motion faster than {T:>2}s known: ATE={np.mean(v):.2f} m")
