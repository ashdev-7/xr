"""Resource measurement as the organizers are likely to do it (FAQ: heap level, not interpreter RSS).

Runs src/main.py in a fresh interpreter under tracemalloc from before the import of main (so the
script's own imports count) and reports: traced heap peak (one traced run), median wall time, CPU time, ms per second of IMU.
usage: python tools/bench/measure.py <input> [--param=value ...]
"""
import os, sys, subprocess, json
CODE = r'''
import tracemalloc, time, sys, os, json
if TRACE: tracemalloc.start()
t0 = time.perf_counter(); c0 = time.process_time()
sys.argv = ["main.py"] + ARGS
sys.path.insert(0, SRC)
import runpy
runpy.run_path(os.path.join(SRC, "main.py"), run_name="__main__")
wall = time.perf_counter() - t0; cpu = time.process_time() - c0
cur, peak = tracemalloc.get_traced_memory() if TRACE else (0, 0)
print(json.dumps(dict(peak_heap_mb=peak / 2**20, wall_s=wall, cpu_s=cpu)))
'''
if __name__ == "__main__":
    inp = sys.argv[1]; extra = sys.argv[2:]
    src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src"))
    out = os.path.join(os.environ.get("TEMP", "/tmp"), "measure_out.txt")
    def once(trace):
        code = CODE.replace("ARGS", repr([inp, out] + extra)).replace("SRC", repr(src)).replace("TRACE", repr(trace))
        r = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True)
        if r.returncode:
            print(r.stderr); sys.exit(1)
        return json.loads(r.stdout.strip().splitlines()[-1])
    # memory and time come from separate runs: tracemalloc slows Python allocation-heavy code ~30x
    m = once(True)
    times = sorted((once(False) for _ in range(3)), key=lambda d: d["cpu_s"])
    m["wall_s"], m["cpu_s"] = times[1]["wall_s"], times[1]["cpu_s"]
    # duration of the IMU log
    ts = [l.split(",")[0] for l in open(os.path.join(inp, "imu0", "data.csv") if os.path.isdir(inp) else inp) if l[:1].isdigit()]
    dur = (int(ts[-1]) - int(ts[0])) * 1e-9
    m.update(duration_s=dur, rows=len(ts), ms_per_s=1000 * m["cpu_s"] / dur, bytes_per_row=m["peak_heap_mb"] * 2**20 / len(ts))
    print(json.dumps(m, indent=1))
