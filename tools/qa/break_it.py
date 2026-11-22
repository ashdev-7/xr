"""Adversarial QA: try to break src/main.py the way a strict evaluator (or a hostile input) would.

Every case runs main.py in a fresh interpreter, exactly as a grader would, with tracemalloc started before
main.py is compiled (strictest memory reading). Checks per case:
  contract  exit code 0, one pose per input row (rows whose timestamp token parses), output timestamps equal
            to the input timestamps exactly (ns), 8 columns, all finite, |q| = 1 +- 1e-6, nothing appended
  resources CPU ms per second of IMU and worst-case traced peak heap
  accuracy  ATE / pre-failure / blind (evo-equivalent SE(3) alignment) where ground truth applies
usage: python tools/qa/break_it.py [case-substring ...]
"""
import io, json, math, os, random, shutil, subprocess, sys, zipfile
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "bench"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import score as S  # noqa: E402

MAIN = os.path.join(ROOT, "src", "main.py")
WORK = os.path.join(ROOT, "data", "qa")
BENCH = os.path.join(ROOT, "data", "bench")
RUNNER = r'''
import tracemalloc, runpy, sys, time, json
if TRACE: tracemalloc.start()
c = time.process_time()
sys.argv = ["main.py"] + ARGS
code = 0
try:
    runpy.run_path(MAIN, run_name="__main__")
except SystemExit as e:
    code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
print("@@" + json.dumps(dict(cpu=time.process_time() - c, peak=tracemalloc.get_traced_memory()[1] if TRACE else 0, code=code)))
'''


def run_main(inp, out, extra=(), trace=True):
    code = RUNNER.replace("ARGS", repr([inp, out] + list(extra))).replace("MAIN", repr(MAIN)).replace("TRACE", repr(trace))
    r = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, timeout=900)
    line = [l for l in r.stdout.splitlines() if l.startswith("@@")]
    if r.returncode or not line:
        return dict(code=r.returncode or 1, err=r.stderr[-600:]), r.stderr
    d = json.loads(line[-1][2:])
    return d, r.stderr


def input_ns(path):
    """Timestamps (ns, exact) of the rows main.py must answer: rows whose first token parses as a number."""
    if os.path.isdir(path):
        for c in ("imu0/data.csv", "mav0/imu0/data.csv", "data.csv"):
            if os.path.isfile(os.path.join(path, c)):
                path = os.path.join(path, c); break
    if path.endswith(".zip"):
        with zipfile.ZipFile(path) as z:
            n = [x for x in z.namelist() if x.endswith("imu0/data.csv")] or [x for x in z.namelist() if x.endswith(".csv")]
            text = z.read(n[0]).decode("utf-8-sig", "replace")
    else:
        text = open(path, encoding="utf-8-sig", errors="replace").read()
    out = []
    for line in text.splitlines():
        s = line.strip()
        if not s or not (s[0].isdigit() or s[0] in "+-."):
            continue
        if "," not in s:
            s = s.replace(";", ",") if ";" in s else ",".join(s.split())
        tok = s.replace(" ", "").split(",")[0].lstrip("+")
        try:  # integer tokens are nanoseconds (EuRoC); decimal tokens are seconds
            out.append(int(tok) if tok.lstrip("-").isdigit() else S.parse_ts_ns(tok))
        except Exception:
            pass
    return out


def contract(out, ns_in):
    if not os.path.isfile(out):
        return "no output file"
    rows = [l.split() for l in open(out) if l.strip() and not l.lstrip().startswith("#")]
    if len(rows) != len(ns_in):
        return f"{len(rows)} poses for {len(ns_in)} input rows"
    for i, (r, n) in enumerate(zip(rows, ns_in)):
        if len(r) != 8:
            return f"row {i}: {len(r)} columns"
        if S.parse_ts_ns(r[0]) != n:
            return f"row {i}: timestamp {r[0]} != input {n}"
        v = [float(x) for x in r[1:]]
        if not all(math.isfinite(x) for x in v):
            return f"row {i}: non-finite"
        qn = math.sqrt(sum(x * x for x in v[3:]))
        if abs(qn - 1) > 1e-6:
            return f"row {i}: |q| = {qn}"
    return "ok"


# ----------------------------------------------------------------------------- case builders
def read_rows(path):
    lines = open(path).read().splitlines()
    return lines[0], [l for l in lines[1:] if l.strip()]


def put(case, text=None, rows=None, header=None, gt_from=None, meta_from=None, gt_shift_ns=0, raw_bytes=None,
        name="data.csv"):
    d = os.path.join(WORK, case)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.join(d, "v", "imu0"))
    p = os.path.join(d, "v", "imu0", name)
    if raw_bytes is not None:
        open(p, "wb").write(raw_bytes)
    else:
        if text is None:
            text = "\n".join(([header] if header else []) + rows) + "\n"
        open(p, "w", newline="").write(text)
    if gt_from:
        tg, pg, qg = S.read_tum(os.path.join(BENCH, gt_from, "gt.tum"))
        with open(os.path.join(d, "gt.tum"), "w") as f:
            for t, p_, q in zip(tg, pg, qg):
                f.write(f"{t + gt_shift_ns * 1e-9:.9f} {p_[0]} {p_[1]} {p_[2]} {q[0]} {q[1]} {q[2]} {q[3]}\n")
    if meta_from:
        m = json.load(open(os.path.join(meta_from, "meta.json")))
        for k in ("t_first", "t_last", "fail_time", "gt_first", "gt_last"):
            m[k] += gt_shift_ns * 1e-9
        m["gaps"] = [[a + gt_shift_ns * 1e-9, b + gt_shift_ns * 1e-9, n] for a, b, n in m["gaps"]]
        json.dump(m, open(os.path.join(d, "v", "meta.json"), "w"))
    return os.path.join(d, "v")


def rot(rows, R):
    out = []
    for l in rows:
        p = l.split(",")
        w = np.array([float(x) for x in p[1:4]]); a = np.array([float(x) for x in p[4:7]])
        w2, a2 = R @ w, R @ a
        out.append(",".join([p[0]] + [repr(float(x)) for x in w2] + [repr(float(x)) for x in a2]))
    return out


def Rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def build_cases():
    C = []  # (name, input path, has_gt, note)
    for base, fl in (("drone", "MH_02_easy"), ("hand", "TUMVI_room2")):
        src = os.path.join(BENCH, fl, "aware_s42")
        hdr, rows = read_rows(os.path.join(src, "imu0", "data.csv"))
        t0 = int(rows[0].split(",")[0])
        C.append((f"{base}_reference", put(f"{base}_reference", rows=rows, header=hdr, gt_from=fl, meta_from=src), True,
                  "unmodified official degradation (aware, seed 42)"))
        # invariances: the answer must not depend on how the IMU is mounted or when the clock started
        for nm, R in (("rot_x90", Rx(90)), ("upside_down", Rx(180)), ("rot_random", Rz(37) @ Rx(-121) @ Rz(64))):
            C.append((f"{base}_{nm}", put(f"{base}_{nm}", rows=rot(rows, R), header=hdr, gt_from=fl, meta_from=src), True,
                      "IMU frame rotated by a fixed rotation (unknown mounting)"))
        z = [",".join([str(int(l.split(",")[0]) - t0 + 1)] + l.split(",")[1:]) for l in rows]
        C.append((f"{base}_clock_from_1ns", put(f"{base}_clock_from_1ns", rows=z, header=hdr, gt_from=fl, meta_from=src,
                                                 gt_shift_ns=-t0 + 1), True, "timestamps start at 1 ns instead of the Unix epoch"))
        C.append((f"{base}_100Hz", put(f"{base}_100Hz", rows=rows[::2], header=hdr, gt_from=fl, meta_from=src), True,
                  "every second row (100 Hz)"))
        up = []
        for a, b in zip(rows[:-1], rows[1:]):
            up.append(a)
            pa, pb = a.split(","), b.split(",")
            ta, tb = int(pa[0]), int(pb[0])
            if tb - ta < 8_000_000:
                up.append(",".join([str((ta + tb) // 2)] + [repr(0.5 * (float(x) + float(y))) for x, y in zip(pa[1:], pb[1:])]))
        up.append(rows[-1])
        C.append((f"{base}_400Hz", put(f"{base}_400Hz", rows=up, header=hdr, gt_from=fl, meta_from=src), True,
                  "midpoint-interpolated 400 Hz"))
        C.append((f"{base}_tab_delim", put(f"{base}_tab_delim", rows=[l.replace(",", "\t") for l in rows], header=hdr.replace(",", "\t"),
                                           gt_from=fl, meta_from=src), True, "tab-separated instead of comma"))
        C.append((f"{base}_semicolon", put(f"{base}_semicolon", rows=[l.replace(",", ";") for l in rows], header=hdr.replace(",", ";"),
                                           gt_from=fl, meta_from=src), True, "semicolon-separated"))
        C.append((f"{base}_extra_cols", put(f"{base}_extra_cols", rows=[l + ",25.31,0" for l in rows], header=hdr + ",temp,flag",
                                            gt_from=fl, meta_from=src), True, "two extra trailing columns"))
        C.append((f"{base}_sci_notation", put(f"{base}_sci_notation", rows=[",".join([l.split(",")[0]] + ["%.6e" % float(x) for x in l.split(",")[1:]]) for l in rows],
                                              header=hdr, gt_from=fl, meta_from=src), True, "values in 1.234560e-03 form"))
        rng = random.Random(5)
        sh = list(rows)
        for i in range(1000, len(sh) - 10, 2000):  # swap neighbouring rows: out-of-order delivery
            sh[i], sh[i + 1] = sh[i + 1], sh[i]
        C.append((f"{base}_swapped_rows", put(f"{base}_swapped_rows", rows=sh, header=hdr, gt_from=fl, meta_from=src), True,
                  "neighbouring rows swapped every 2000 rows"))
        jump = [l if i < len(rows) // 2 else ",".join([str(int(l.split(",")[0]) + 100 * 10**9)] + l.split(",")[1:])
                for i, l in enumerate(rows)]
        C.append((f"{base}_clock_jump_100s", put(f"{base}_clock_jump_100s", rows=jump, header=hdr), False,
                  "clock jumps +100 s mid-file (no ground truth)"))
        reset = [l if i < len(rows) // 2 else ",".join([str(int(l.split(",")[0]) - 50 * 10**9)] + l.split(",")[1:])
                 for i, l in enumerate(rows)]
        C.append((f"{base}_clock_reset", put(f"{base}_clock_reset", rows=reset, header=hdr), False,
                  "clock jumps back 50 s mid-file"))
        n10 = int(10 / 0.005)
        dead = [",".join([str(t0 - (n10 - i) * 5_000_000)] + ["0"] * 6) for i in range(n10)]
        C.append((f"{base}_10s_dead_start", put(f"{base}_10s_dead_start", rows=dead + rows, header=hdr, gt_from=fl), False,
                  "10 s of all-zero rows before the data"))
        k = len(rows) // 2
        cut = rows[:k] + rows[k + 2000:]
        C.append((f"{base}_10s_dropout", put(f"{base}_10s_dropout", rows=cut, header=hdr, gt_from=fl, meta_from=src), True,
                  "one 10 s signal loss mid-flight"))
        bad0 = ["0," + rows[0].split(",", 1)[1]] + rows[1:]
        C.append((f"{base}_first_ts_zero", put(f"{base}_first_ts_zero", rows=bad0, header=hdr, gt_from=fl, meta_from=src), True,
                  "first row's timestamp corrupted to 0"))
        badf = [str(t0 + 10**15) + "," + rows[0].split(",", 1)[1]] + rows[1:]
        C.append((f"{base}_first_ts_future", put(f"{base}_first_ts_future", rows=badf, header=hdr, gt_from=fl, meta_from=src), True,
                  "first row's timestamp corrupted to +11.6 days"))
        af = [",".join([l.split(",")[0]] + l.split(",")[4:7] + l.split(",")[1:4]) for l in rows]
        C.append((f"{base}_ambiguous_hdr_accel_first", put(f"{base}_ambiguous_hdr_accel_first", rows=af,
                  header="timestamp,acceleration_x,acceleration_y,acceleration_z,angular_velocity_x,angular_velocity_y,angular_velocity_z",
                  gt_from=fl, meta_from=src), True, "accel-first data, header names both starting with 'a'"))
        C.append((f"{base}_cp1252_header", put(f"{base}_cp1252_header",
                                               raw_bytes=("#timestamp [ns],w_x \xb0/s,w_y,w_z,a_x \xb5,a_y,a_z\n".encode("cp1252")
                                                          + "\n".join(rows).encode() + b"\n"),
                                               gt_from=fl, meta_from=src), True, "non-UTF-8 bytes in the header"))
    # degenerate inputs (no ground truth)
    hdr, rows = read_rows(os.path.join(BENCH, "MH_02_easy", "aware_s42", "imu0", "data.csv"))
    C.append(("empty_file", put("empty_file", text=""), False, "0 bytes"))
    C.append(("header_only", put("header_only", text=hdr + "\n"), False, "header, no rows"))
    C.append(("one_row", put("one_row", rows=rows[:1], header=hdr), False, "1 row"))
    C.append(("three_rows", put("three_rows", rows=rows[:3], header=hdr), False, "3 rows"))
    C.append(("all_zero", put("all_zero", rows=[r.split(",")[0] + ",0,0,0,0,0,0" for r in rows], header=hdr), False, "every row zero"))
    C.append(("all_nan", put("all_nan", rows=[r.split(",")[0] + ",nan,nan,nan,nan,nan,nan" for r in rows], header=hdr), False, "every row NaN"))
    t0 = int(rows[0].split(",")[0])
    still = [f"{t0 + i * 5_000_000},0,0,0,0,0,9.81" for i in range(12000)]
    C.append(("perfect_still_60s", put("perfect_still_60s", rows=still, header=hdr), False, "noise-free stationary, 60 s"))
    C.append(("trailing_comma", put("trailing_comma", rows=[r + "," for r in rows], header=hdr), False, "trailing comma on every row"))
    C.append(("plus_signs", put("plus_signs", rows=["+" + r for r in rows], header=hdr), False, "leading + on timestamps"))
    C.append(("no_final_newline", put("no_final_newline", text="\n".join([hdr] + rows)), False, "file does not end with a newline"))
    d = os.path.join(WORK, "zip_odd"); shutil.rmtree(d, ignore_errors=True); os.makedirs(d)
    zp = os.path.join(d, "eval data (1).zip")
    with zipfile.ZipFile(zp, "w") as z:
        z.writestr("__MACOSX/._data.csv", b"\x00\x05\x16\x07junk")
        z.writestr("readme.txt", "x")
        z.writestr("Eval Ünïcode/mav0/imu0/data.csv", "\n".join([hdr] + rows) + "\n")
    C.append(("zip_odd", zp, False, "zip: spaces/unicode in names, __MACOSX junk, extra files, mav0 nesting"))
    return C


def long_logs():
    """30 and 60 min logs: raw EuRoC flights concatenated (continuous clock), then the official degradation."""
    import pandas as pd
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import degrade_imu_official as official
    out = []
    raws = [os.path.join(ROOT, "data", "raw", "euroc", f, "imu0.csv") for f in
            ("MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult", "V1_01_easy",
             "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium", "V2_03_difficult")]
    dfs = [pd.read_csv(p, header=official.detect_header(p)) for p in raws]
    for minutes in (30, 60):
        parts, t, k = [], None, 0
        total = 0.0
        while total < minutes * 60:
            df = dfs[k % len(dfs)].copy(); k += 1
            c0 = df.columns[0]
            ts = df[c0].to_numpy(np.int64)
            ts = ts - ts[0] + (0 if t is None else t + 5_000_000)
            df[c0] = ts + (1403636579758555392 if t is None else 0)
            if t is not None:
                df[c0] = ts
            t = int(df[c0].iloc[-1]); total = (t - int(parts[0][parts[0].columns[0]].iloc[0]) if parts else 0) * 1e-9
            parts.append(df)
        raw = pd.concat(parts, ignore_index=True)
        cols = list(raw.columns)
        re = raw[[cols[0]] + cols[4:7] + cols[1:4]]
        o, _ = official.degrade(re, dict(official.CONFIG), 42)
        o = o[cols]
        d = os.path.join(WORK, f"long_{minutes}min", "v", "imu0")
        shutil.rmtree(os.path.dirname(os.path.dirname(d)), ignore_errors=True); os.makedirs(d)
        o.to_csv(os.path.join(d, "data.csv"), index=False)
        out.append((f"long_{minutes}min", os.path.dirname(d), False, f"{minutes} min of real EuRoC IMU, official degradation"))
    return out


def main():
    sel = sys.argv[1:]
    cases = build_cases()
    if not sel or any("long" in s for s in sel):
        cases += long_logs()
    if sel:
        cases = [c for c in cases if any(s in c[0] for s in sel)]
    ref = {}
    rows_out = []
    for name, inp, has_gt, note in cases:
        out = os.path.join(WORK, name + "_out.txt")
        if os.path.exists(out):
            os.remove(out)
        r, err = run_main(inp, out)
        if r.get("code") == 0:  # timing from a separate untraced run (tracemalloc slows Python ~30x)
            rt, _ = run_main(inp, out + ".t", trace=False)
            r["cpu"] = rt.get("cpu", float("nan"))
            if os.path.exists(out + ".t"):
                os.remove(out + ".t")
        ns_in = input_ns(inp)
        c = contract(out, ns_in) if r.get("code") == 0 else f"exit {r.get('code')}: {r.get('err', err)[-300:]}"
        dur = (max(ns_in) - min(ns_in)) * 1e-9 if len(ns_in) > 1 else 0.0
        msps = 1000 * r["cpu"] / dur if dur > 1 and "cpu" in r else float("nan")
        acc = {}
        if has_gt and c == "ok" and os.path.isfile(os.path.join(inp, "meta.json")):
            try:
                acc = S.score(out, inp)
            except Exception as e:  # noqa: BLE001
                acc = dict(err=str(e)[:80])
        fb = "fallback" if "stand-still" in err else ""
        rows_out.append(dict(case=name, note=note, contract=c, ms_per_s=msps, peak_mb=r.get("peak", 0) / 2**20,
                             rows=len(ns_in), ate=acc.get("ate"), wp=acc.get("wp"), blind=acc.get("blind"),
                             drop=acc.get("drop"), fallback=fb))
        a = acc.get("ate")
        print(f"{'PASS' if c == 'ok' else 'FAIL'} {name:26s} rows {len(ns_in):7d}  {msps:6.2f} ms/s  peak {r.get('peak', 0) / 2**20:5.2f} MB"
              + (f"  ATE {a:.3f} wp {acc['wp']:.3f} blind {acc['blind']:.3f}" if a is not None else "")
              + (f"  [{fb}]" if fb else "") + ("" if c == "ok" else f"  <-- {c[:200]}"), flush=True)
    json.dump(rows_out, open(os.path.join(ROOT, "results", "qa_break_it.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
