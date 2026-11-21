"""One-time refactor: cut the single-file estimator into main.py + stage files at top-level boundaries.

Why: CPython parses a whole file at once; for the 1,350-line estimator that costs ~7 MB of traced heap (7.9 MB
peak on Python 3.13 if memory is traced from interpreter start). Stage files compiled one at a time peak at
<= 1.5 MB each. The stage code is executed into main.py's own namespace, so every function and global is the
same object as before and the output is bit-identical by construction.
usage: python tools/split_main.py   (reads src/main.py, rewrites it and writes src/pg_*.py)
"""
import os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SRC = os.path.join(ROOT, "src")
L = open(os.path.join(SRC, "main.py"), encoding="utf-8").read().split("\n")


def idx(prefix):
    hits = [i for i, l in enumerate(L) if l.startswith(prefix)]
    assert len(hits) == 1, (prefix, hits)
    return hits[0]


sv, q, ia, ch, wo, mn = (idx("def survey("), idx("def qmul("), idx("def initial_attitude("),
                         idx("def chol_solve6("), idx("def write_output("), idx("def main(argv)"))
STAGES = [
    ("pg_ingest.py", "pass 1: survey of the input and the single streaming pass (integrity, jamming meter, "
                     "preintegration, rest statistics)", L[sv:q]),
    ("pg_math.py", "quaternion / 3x3 helpers and the attitude error-state Kalman filter (class Att)", L[q:ia]),
    ("pg_attitude.py", "initial attitude, attitude filter + RTS smoother", L[ia:ch]),
    ("pg_translation.py", "translation: per-axis Kalman filter + RTS smoother on 10 Hz knots", L[ch:wo]),
    ("pg_output.py", "pass 2 (one pose per input row) and the pipeline driver run()", L[wo:mn]),
]
for name, what, body in STAGES:
    while body and not body[-1].strip():
        body.pop()
    head = [f"# PathGuard stage file: {what}.",
            "# Executed by main.py into main.py's own namespace (see _load_stages there), not imported as a module:",
            "# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.",
            "# Do not run directly.", ""]
    open(os.path.join(SRC, name), "w", encoding="utf-8", newline="\n").write("\n".join(head + body) + "\n")

LOADER = '''

# ----------------------------------------------------------------------------- stage loader
# The pipeline lives in five stage files next to this one. CPython parses a whole file at once, and parsing the
# estimator as a single 1,350-line file costs ~7 MB of heap (7.9 MB peak on Python 3.13 when memory is traced
# from interpreter start). Compiling the stages one at a time keeps that transient cost at <= 1.5 MB. Each stage
# is executed into THIS module's namespace, so the code behaves exactly as one file. The stage files are found
# by path (never via sys.path, which "python -I" does not extend). If they are missing, main() still writes a
# well-formed stand-still trajectory (one pose per input row) and says so on stderr.
STAGES = ("pg_ingest.py", "pg_math.py", "pg_attitude.py", "pg_translation.py", "pg_output.py")


def _stage_dir():
    here = globals().get("__file__") or (sys.argv[0] if sys.argv and sys.argv[0] else "")
    cands = [os.path.dirname(os.path.abspath(here))] if here else []
    cands += [os.getcwd(), os.path.join(os.getcwd(), "src")]
    for d in cands:
        if all(os.path.isfile(os.path.join(d, s)) for s in STAGES):
            return d
    raise FileNotFoundError("stage files " + ", ".join(STAGES) + " not found next to main.py")


def _load_stages():
    d = _stage_dir()
    g = globals()
    for s in STAGES:
        path = os.path.join(d, s)
        with open(path, encoding="utf-8") as f:
            code = compile(f.read(), path, "exec")
        exec(code, g)
        del code


try:
    _load_stages()
    LOAD_ERROR = None
except Exception as _e:  # noqa: BLE001  (any failure -> stand-still output, never a crash)
    LOAD_ERROR = f"{type(_e).__name__}: {_e}"
'''

head = L[:sv]
while head and not head[-1].strip():
    head.pop()
tail = L[mn:]
out = "\n".join(head) + LOADER + "\n\n" + "\n".join(tail)
out = out.replace('''def main(argv):
    pos_args = []''', '''def main(argv):
    if LOAD_ERROR is not None:
        print(f"pathguard: cannot load the estimator ({LOAD_ERROR}); writing a stand-still trajectory", file=sys.stderr)
        pos = [a for a in argv[1:] if not (a.startswith("--") and "=" in a)]
        fallback(pos[0] if pos else None, pos[1] if len(pos) > 1 else "estimated_trajectory.txt")
        return
    pos_args = []''')
open(os.path.join(SRC, "main.py"), "w", encoding="utf-8", newline="\n").write(out)
print("main.py", out.count("\n") + 1, "lines;", ", ".join(f"{n} {len(b)}" for n, _, b in STAGES))
