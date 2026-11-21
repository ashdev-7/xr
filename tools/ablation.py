"""Ablation on held-out TEST flights (both degradation styles)."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from tune import evaluate
from bench import TEST
cfgs = [
 ("PathGuard (full)", {}),
 ("- backward attitude pass", {"backward": 0}),
 ("- integrity layer", {"repair": 0}),
 ("- noise-adaptive trust", {"adapt": 0}),
 ("- motion prior (loose)", {"sig_v_h": 100, "sig_v_z": 100, "sig_p_h": 1000, "sig_p_z": 1000}),
 ("blind: constant velocity", {"blind_tau": 1e6}),
 ("blind: decaying velocity 1s", {"blind_tau": 1.0}),
]
print(f"{'config':32s}{'ATE':>8s}{'WP':>8s}{'drop':>8s}{'blind5s':>9s}{'RPE1s':>8s}")
for name, p in cfgs:
    m = evaluate(p, TEST)
    print(f"{name:32s}{m['ate']:8.2f}{m['wp']:8.2f}{m['drop']:8.2f}{m['blind']:9.2f}{m['rpe1s']:8.2f}", flush=True)
