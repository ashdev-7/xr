"""Assemble the Track 03 submission bundle exactly as specified (organizer deck, slide 17):

    <CollegeName_TeamName>.zip
    └── <CollegeName_TeamName>/
        ├── estimated_trajectory.txt   generated here from the evaluation IMU file
        ├── metadata.json              measured runtime / memory / compute specs
        └── src/
            ├── main.py                the estimator (standalone, stdlib only)
            ├── README.md              how to run
            └── Dockerfile

usage: python tools/make_bundle.py <CollegeName_TeamName> <evaluation imu0/data.csv | folder | zip>
The output contract (one pose per row, exact timestamps, unit quaternions) is verified before zipping.
"""
import json, os, platform, shutil, subprocess, sys, time, zipfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tests"))


def main(team, imu):
    out = os.path.join(ROOT, "dist", team)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(os.path.join(out, "src"))
    for f in ("main.py", "Dockerfile"):
        shutil.copy(os.path.join(ROOT, "src", f), os.path.join(out, "src", f))
    shutil.copy(os.path.join(ROOT, "README.md"), os.path.join(out, "src", "README.md"))
    traj = os.path.join(out, "estimated_trajectory.txt")
    main_py = os.path.join(out, "src", "main.py")
    # run (timed, no profiler)
    t0 = time.perf_counter()
    r = subprocess.run([sys.executable, "-I", main_py, imu, traj, "--verbose=1"], capture_output=True, text=True)
    wall = time.perf_counter() - t0
    if r.returncode:
        sys.exit("estimator failed:\n" + r.stderr)
    diag = r.stderr.strip().splitlines()
    # CPU time of the estimator itself (fresh interpreter, no profiler, median of 3)
    code_t = ("import sys,time; sys.path.insert(0,%r); import main as M; c=time.process_time(); w=time.perf_counter(); "
              "M.run(%r, %r); print(time.process_time()-c, time.perf_counter()-w)") % (os.path.dirname(main_py), imu, traj + ".tmp")
    tm = sorted(tuple(map(float, subprocess.run([sys.executable, "-I", "-c", code_t], capture_output=True, text=True).stdout.split()))
                for _ in range(3))[1]
    # traced heap of the estimator's data structures (run in a fresh interpreter)
    code = ("import tracemalloc,sys; sys.path.insert(0,%r); import main as M; tracemalloc.start(); "
            "M.run(%r, %r); print(tracemalloc.get_traced_memory()[1])") % (os.path.dirname(main_py), imu, traj + ".tmp")
    heap = int(subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True).stdout)
    os.remove(traj + ".tmp")
    # contract check against the input
    from test_contract import ns_of
    sys.path.insert(0, os.path.dirname(main_py))
    import importlib
    M = importlib.import_module("main")
    opener, src = M.resolve_input(imu)
    ns_in = []
    with opener() as f:
        for line in f:
            s = line.strip()
            if s and (s[0].isdigit() or s[0] in "+-."):
                tok = s.split(",", 1)[0].strip()
                try:
                    ns_in.append(int(tok) if "." not in tok else ns_of(tok))
                except ValueError:
                    pass
    rows = [l.split() for l in open(traj) if l.strip() and not l.startswith("#")]
    assert len(rows) == len(ns_in), f"{len(rows)} poses for {len(ns_in)} rows"
    assert all(len(x) == 8 for x in rows), "column count"
    assert all(ns_of(x[0]) == n for x, n in zip(rows, ns_in)), "timestamps differ from input"
    dur = (ns_in[-1] - ns_in[0]) * 1e-9
    meta = dict(
        team=team, track="03 Path Finder: Trajectory Estimation Under Sensor Degradation",
        method="PathGuard: integrity layer + packet-loss jamming meter + error-state Kalman/RTS attitude smoother "
               "+ per-axis Kalman/RTS translation smoother with per-platform motion prior, rest updates and "
               "rotor-drag velocity measurement (multirotors)",
        entrypoint="src/main.py (pure Python standard library)",
        input=os.path.basename(src), input_rows=len(ns_in), output_poses=len(rows), imu_duration_s=round(dur, 3),
        cpu_time_s=round(tm[0], 3), ms_per_s_of_imu=round(1000 * tm[0] / max(dur, 1e-9), 3),
        wall_time_s_including_interpreter_start=round(wall, 3),
        peak_heap_mb_estimator=round(heap / 2**20, 3), threads=1, network="none (offline)",
        python=platform.python_version(), compute=f"{platform.processor() or platform.machine()}, {platform.system()} {platform.release()}",
        diagnostics=[l for l in diag if l.startswith(("platform", "jamming", "integrity"))])
    json.dump(meta, open(os.path.join(out, "metadata.json"), "w"), indent=1)
    z = os.path.join(ROOT, "dist", team + ".zip")
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for dp, _dn, fns in os.walk(out):
            for fn in fns:
                if fn.endswith(".pyc"):
                    continue
                p = os.path.join(dp, fn)
                zf.write(p, os.path.relpath(p, os.path.join(ROOT, "dist")))
    print(json.dumps(meta, indent=1))
    print("bundle:", z)
    with zipfile.ZipFile(z) as zf:
        for n in zf.namelist():
            print("  ", n)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
