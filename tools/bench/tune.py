"""Coordinate-descent tuning on TRAIN flights only; TEST is reported, never optimized.

Objective (lower is better), mirroring the official 30/20/10 accuracy weights, each metric
normalized by the stand-still baseline of the same variant so that every flight counts equally:
    J = mean_v [ 0.5 wp/wp_still + 1/3 blind/blind_still + 1/6 drop/drop_still ]

usage: python tools/bench/tune.py [--rounds 2] [--params a,b,c] [--seeds 42,1] [--start --k=v ...]
"""
import json, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run import run  # noqa: E402

SPLITS = {  # split by flight; TEST flights are never used to choose parameters
    "handheld": (["TUMVI_room1", "TUMVI_room3", "TUMVI_room5"], ["TUMVI_room2", "TUMVI_room4", "TUMVI_room6"]),
    "rotor": (["MH_01_easy", "MH_03_medium", "MH_05_difficult", "V1_01_easy", "V2_02_medium"],
              ["MH_02_easy", "MH_04_difficult", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_03_difficult"]),
}
SPLITS["all"] = (SPLITS["handheld"][0] + SPLITS["rotor"][0], SPLITS["handheld"][1] + SPLITS["rotor"][1])
# final fit (after the method was validated on held-out flights): every public flight of the class; the
# "test" score of these splits is in-sample and is NOT a generalization estimate
for _c in ("rotor", "handheld"):
    SPLITS[_c + "_final"] = (SPLITS[_c][0] + SPLITS[_c][1], SPLITS[_c][0] + SPLITS[_c][1])
W = dict(wp=0.5, blind=1 / 3, drop=1 / 6)
_still = {}


def still_ref(flights, seeds):
    key = (tuple(flights), tuple(seeds))
    if key not in _still:
        _, res = run("still", [], flights, ("asis", "aware"), seeds, tag="still_ref", quiet=True)
        _still[key] = {(r["flight"], r["scen"], r["seed"]): r for r in res}
    return _still[key]


def objective(params, flights, seeds, tag="tune"):
    ref = still_ref(flights, seeds)
    _, res = run("pg", params, flights, ("asis", "aware"), seeds, tag=tag, quiet=True)
    js = []
    for r in res:
        s = ref[(r["flight"], r["scen"], r["seed"])]
        j = sum(w * (r[k] / s[k] if np.isfinite(r[k]) and np.isfinite(s[k]) and s[k] > 0 else 1.0) for k, w in W.items())
        js.append(j)
    return float(np.mean(js)), res


def fmt(p):
    return " ".join(f"--{k}={v:.6g}" for k, v in p.items())


if __name__ == "__main__":
    sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
    import main as M
    a = sys.argv[1:]
    rounds, names, seeds, start, cls = 2, None, (42, 1), {}, "all"
    while a:
        x = a.pop(0)
        if x == "--rounds": rounds = int(a.pop(0))
        elif x == "--params": names = a.pop(0).split(",")
        elif x == "--seeds": seeds = tuple(int(s) for s in a.pop(0).split(","))
        elif x == "--cls": cls = a.pop(0)
        elif x.startswith("--") and "=" in x:
            k, v = x[2:].split("=", 1); start[k] = float(v)
    bench = os.path.join(HERE, "..", "..", "data", "bench")
    have = set(os.listdir(bench))
    train = [f for f in SPLITS[cls][0] if f in have]
    test = [f for f in SPLITS[cls][1] if f in have]
    print("TRAIN:", train, "\nTEST:", test, flush=True)
    names = names or ["sig_v_h", "sig_p_h", "sig_v_z", "sig_p_z", "sig_a", "ba_rw", "ba_k", "ba_kz", "sig_c",
                      "tilt_sig", "tilt_k", "bg0_sig", "bg_rw", "bg_k", "sig_gap"] + (["drag_sig", "drag_mu"] if cls.startswith("rotor") else [])
    preset = M.PRESETS[1 if cls.startswith("rotor") else 0]
    cur = {k: float(start.get(k, preset.get(k, M.P[k]))) for k in names}
    cur.update({k: v for k, v in start.items()})
    base = lambda p: [f"--{k}={v:.6g}" for k, v in p.items()]
    t0 = time.time()
    J, _ = objective(base(cur), train, seeds)
    print(f"start J_train={J:.4f}  ({time.time()-t0:.0f}s)  {fmt(cur)}", flush=True)
    log = [dict(J=J, p=dict(cur))]
    for rd in range(rounds):
        steps = (0.5, 2.0) if rd == 0 else (0.7, 1.4)
        for k in names:
            best = (J, cur[k])
            for f in steps:
                trial = dict(cur); trial[k] = cur[k] * f
                Jt, _ = objective(base(trial), train, seeds)
                print(f"  r{rd} {k}={trial[k]:.4g}: J={Jt:.4f}", flush=True)
                if Jt < best[0] - 0.002:
                    best = (Jt, trial[k])
            if best[1] != cur[k]:
                cur[k] = best[1]; J = best[0]
                print(f"  -> {k}={cur[k]:.4g}  J_train={J:.4f}", flush=True)
                log.append(dict(J=J, p=dict(cur)))
    Jte, _ = objective(base(cur), test, (42, 1, 2), tag="tune_test")
    print(f"FINAL J_train={J:.4f} J_test={Jte:.4f}\n{fmt(cur)}", flush=True)
    os.makedirs(os.path.join(HERE, "..", "..", "results"), exist_ok=True)
    json.dump(dict(final=cur, J_train=J, J_test=Jte, log=log, train=train, test=test),
              open(os.path.join(HERE, "..", "..", "results", f"tune_{cls}.json"), "w"), indent=1)
