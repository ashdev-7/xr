"""Output-contract tests: every plausible input variant must give exactly one valid pose per input row.

Checks per case: exit code 0, row count == input data rows, timestamps equal the input timestamps converted
to seconds (compared exactly as integers of ns), 8 finite columns, unit quaternions.
usage: python tests/test_contract.py [path to a degraded EuRoC imu0/data.csv]
"""
import math, os, shutil, subprocess, sys, tempfile, zipfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MAIN = os.path.join(ROOT, "src", "main.py")
DEFAULT = os.path.join(ROOT, "data", "bench", "V1_02_medium", "asis_s42", "imu0", "data.csv")


def ns_of(tok):
    if "." in tok:
        a, b = tok.split(".")
        return int(a) * 10**9 + int((b + "000000000")[:9])
    return int(tok) * 10**9


def check(name, inp, expect_ns, args=(), tmp=None):
    out = os.path.join(tmp, name + ".txt")
    r = subprocess.run([sys.executable, "-I", MAIN, inp, out, *args], capture_output=True, text=True)
    if r.returncode:
        return f"FAIL {name}: exit {r.returncode}: {r.stderr.strip().splitlines()[-1] if r.stderr.strip() else ''}"
    rows = [l.split() for l in open(out) if l.strip() and not l.startswith("#")]
    if len(rows) != len(expect_ns):
        return f"FAIL {name}: {len(rows)} poses for {len(expect_ns)} input rows"
    for i, (row, e) in enumerate(zip(rows, expect_ns)):
        if len(row) != 8:
            return f"FAIL {name}: row {i} has {len(row)} columns"
        if ns_of(row[0]) != e:
            return f"FAIL {name}: row {i} timestamp {row[0]} != {e} ns"
        v = [float(x) for x in row[1:]]
        if not all(math.isfinite(x) for x in v):
            return f"FAIL {name}: row {i} non-finite"
        if abs(math.sqrt(sum(x * x for x in v[3:])) - 1) > 1e-6:
            return f"FAIL {name}: row {i} quaternion norm"
    return f"ok   {name} ({len(rows)} rows)"


def main(src):
    lines = open(src).read().splitlines()
    header, data = lines[0], [l for l in lines[1:] if l.strip()]
    ns = [int(l.split(",")[0]) for l in data]
    tmp = tempfile.mkdtemp()
    res = []

    def write(name, text, mode="w", newline="\n"):
        p = os.path.join(tmp, name)
        with open(p, mode, newline=newline, encoding="utf-8") as f:
            f.write(text)
        return p

    full = "\n".join([header] + data) + "\n"
    res.append(check("standard", write("std.csv", full), ns, tmp=tmp))
    res.append(check("no_header", write("nohdr.csv", "\n".join(data) + "\n"), ns, tmp=tmp))
    res.append(check("crlf", write("crlf.csv", full, newline="\r\n"), ns, tmp=tmp))
    res.append(check("bom", write("bom.csv", "﻿" + full, newline="\n"), ns, tmp=tmp))
    res.append(check("spaces", write("sp.csv", "\n".join([header] + [l.replace(",", ", ") for l in data]) + "\n"), ns, tmp=tmp))
    # accel-first columns (header names say so) and without header (magnitude detection)
    sw = [",".join([p[0]] + p[4:7] + p[1:4]) for p in (l.split(",") for l in data)]
    hs = header.split(","); hsw = ",".join([hs[0]] + hs[4:7] + hs[1:4])
    res.append(check("accel_first_hdr", write("af.csv", "\n".join([hsw] + sw) + "\n"), ns, tmp=tmp))
    res.append(check("accel_first_nohdr", write("af2.csv", "\n".join(sw) + "\n"), ns, tmp=tmp))
    # accel in g
    g = []
    for l in data:
        p = l.split(",")
        g.append(",".join(p[:4] + [repr(float(x) / 9.80665) for x in p[4:7]]))
    res.append(check("accel_in_g", write("g.csv", "\n".join([header] + g) + "\n"), ns, tmp=tmp))
    # seconds timestamps (float)
    secs = [f"{int(l.split(',')[0]) // 10**9}.{int(l.split(',')[0]) % 10**9:09d}," + l.split(",", 1)[1] for l in data]
    res.append(check("seconds_ts", write("sec.csv", "\n".join(secs) + "\n"), ns, tmp=tmp))
    # NaN rows and duplicated rows
    nn = list(data)
    for i in range(500, len(nn), 997):
        p = nn[i].split(","); nn[i] = ",".join([p[0]] + ["nan"] * 6)
    res.append(check("nan_rows", write("nan.csv", "\n".join([header] + nn) + "\n"), ns, tmp=tmp))
    dd, dns = [], []
    for i, l in enumerate(data):
        dd.append(l); dns.append(ns[i])
        if i % 251 == 0:
            dd.append(l); dns.append(ns[i])
    res.append(check("duplicates", write("dup.csv", "\n".join([header] + dd) + "\n"), dns, tmp=tmp))
    # garbage values in a few rows (non-numeric), short rows
    gb = list(data)
    for i in range(700, len(gb), 1501):
        p = gb[i].split(","); gb[i] = p[0] + ",x,y,z,1,2"
    res.append(check("garbage_rows", write("gb.csv", "\n".join([header] + gb) + "\n"), ns, tmp=tmp))
    # short files
    res.append(check("short_300", write("s300.csv", "\n".join([header] + data[:300]) + "\n"), ns[:300], tmp=tmp))
    res.append(check("short_40", write("s40.csv", "\n".join([header] + data[:40]) + "\n"), ns[:40], tmp=tmp))
    # file ends at failure (no blind rows): still exactly one pose per row, nothing appended
    alive = [l for l in data if any(float(x) != 0 for x in l.split(",")[1:])]
    res.append(check("no_blind_rows", write("nb.csv", "\n".join([header] + alive) + "\n"),
                     [int(l.split(",")[0]) for l in alive], tmp=tmp))
    # dataset folder, mav0 folder and zip
    d1 = os.path.join(tmp, "ds", "imu0"); os.makedirs(d1); shutil.copy(src, os.path.join(d1, "data.csv"))
    res.append(check("folder", os.path.join(tmp, "ds"), ns, tmp=tmp))
    d2 = os.path.join(tmp, "ds2", "mav0", "imu0"); os.makedirs(d2); shutil.copy(src, os.path.join(d2, "data.csv"))
    res.append(check("folder_mav0", os.path.join(tmp, "ds2"), ns, tmp=tmp))
    zp = os.path.join(tmp, "eval.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(src, "team_data/imu0/data.csv")
    res.append(check("zip", zp, ns, tmp=tmp))
    for r in res:
        print(r)
    shutil.rmtree(tmp, ignore_errors=True)
    bad = [r for r in res if r.startswith("FAIL")]
    print(f"{len(res) - len(bad)}/{len(res)} passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT))
