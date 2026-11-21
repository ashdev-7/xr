#!/usr/bin/env python3
"""PathGuard: trust-aware inertial navigation under progressive sensor degradation.

Samsung PRISM XR Hackathon 2026, Track 03 (Path Finder).

    python main.py [imu0/data.csv | dataset folder | dataset .zip] [estimated_trajectory.txt] [--param=value ...]

Input : EuRoC-format IMU CSV (#timestamp[ns], w_x, w_y, w_z [rad/s], a_x, a_y, a_z [m/s^2]).
Output: TUM trajectory (timestamp[s] tx ty tz qx qy qz qw), exactly one pose per input row, including
        duplicated rows and the dead rows of the 5 s after the IMU fails. Nothing is appended.

Pure Python standard library (no numpy, nothing to install). The input is streamed twice; memory grows
with 40 ms bins and 10 Hz knots, never with the number of rows. This file is the entry point; the pipeline is
in the stage files pg_*.py next to it, compiled one at a time to keep peak memory low (see _load_stages).

  pass 1       survey (header, column order, units) + one streaming pass: integrity checks (NaN, dead,
               frozen/duplicate rows, six-channel spikes), jamming meter (packet loss, noise levels),
               preintegration into bins, rest statistics
  platform     rotor-vibration signature -> rotorcraft or handheld motion prior (learned per class)
  attitude     error-state Kalman filter on [dtheta (world), gyro bias] + error-state RTS smoother;
               gravity-direction updates, zero-angular-rate updates at rest
  translation  per-axis Kalman filter + RTS smoother on 10 Hz knots, x = [p, v, accel bias, centre],
               continuous-time motion prior, zero-velocity updates at rest, rotor-drag velocity
               measurement for multirotors; the blind window is the same smoother with no data
  pass 2       one pose per input row: attitude interpolated between bins, position by cubic Hermite
"""
import math
import os
import sys
from array import array

# ----------------------------------------------------------------------------- parameters
P = dict(
    bin_dt=0.04,        # attitude/preintegration bin [s] (25 Hz: -24 % runtime, -28 % heap for ~0.3 % ATE)
    knot_dt=0.1,        # translation smoother step [s]
    # attitude
    tilt_sig=0.5,       # gravity-direction measurement noise per update [rad]
    tilt_k=4.0,         # extra tilt noise per | |f| - g | [rad per m/s^2]
    tilt_gate=3.0,      # skip gravity update if | |f| - g | exceeds this [m/s^2]
    tilt_every=1,       # gravity update every N bins
    gyro_floor=2e-3,    # minimum gyro white noise per sample [rad/s]
    bg_rw=2e-5,         # base gyro bias random walk [rad/s/sqrt(s)]
    bg_k=0.002,         # extra gyro bias RW per unit severity [rad/s/sqrt(s)] (script: 6e-4 aware, ~1e-3..1e-2 as-is)
    gap_gyro_k=10.0,    # gyro noise inflation inside gaps
    gap_alpha=2.0,      # unknown angular acceleration inside data gaps [rad/s^2] (rotation uncertainty ~ alpha L^1.5)
    bg0_sig=0.05,       # initial gyro bias uncertainty [rad/s] (EuRoC raw bias reaches ~0.08)
    # translation (per-second parameterisation, independent of knot_dt)
    sig_a=0.4,          # accel noise per knot [m/s^2] (scaled by sqrt(noise ratio))
    sig_gap=3.0,        # accel uncertainty in the blind window after the failure [m/s^2]
    sig_gap_in=1.5,     # accel uncertainty inside data gaps (dropouts) [m/s^2]
    ba_rw=5e-4,         # base accel bias random walk [m/s^2/sqrt(s)]
    ba_k=0.01,          # extra accel bias RW per unit severity [m/s^2/sqrt(s)] (script: 0.01 aware)
    ba_kz=0.015,        # extra vertical bias RW per unit severity (1.5e-3 scale drift x g)
    sig_c=0.2,          # workspace-centre random walk [m/sqrt(s)]
    sig_v_h=0.3,        # speed prior, horizontal [m/s]
    sig_v_z=0.1,        # speed prior, vertical [m/s]
    sig_p_h=1.0,        # workspace prior, horizontal [m]
    sig_p_z=0.5,        # workspace prior, vertical [m]
    blind_tau=-1.0,     # blind forecast: <0 smoother prior, 0 hold, >0 decaying velocity time constant [s]
    att_tau=0.3,        # blind attitude: 0 hold, >0 last body rate decaying with this time constant [s]
    # integrity
    spike_k=8.0,        # spike threshold in robust sigmas (must hold on all 6 channels)
    # still detection (zero-velocity / zero-angular-rate updates), 0.5 s windows
    still_g=0.04,       # max gyro std [rad/s]
    still_a=1.0,        # max accel std [m/s^2]
    still_dg=0.3,       # max | |mean f| - g | [m/s^2]
    zupt_sig=0.02,      # zero-velocity pseudo-measurement noise [m/s]
    zaru_sig=0.002,     # zero-angular-rate measurement noise floor [rad/s]
    still_tilt=0.02,    # gravity-direction noise while still [rad]
    # rotor drag (rotorcraft only): the body-horizontal specific force of a multirotor is -mu * v_body
    drag=1,             # use the rotor-drag velocity measurement when the platform is a rotorcraft
    drag_mu=0.18,       # linear drag coefficient [1/s] (fitted on TRAIN flights: 0.18-0.20)
    drag_sig=1.2,       # velocity noise of one 10 Hz drag measurement [m/s]
    drag_vib=0.4,       # rotors spinning and airborne when accel vibration exceeds this [m/s^2]
    # platform: -1 auto (rotor vibration signature), 0 handheld/ground, 1 rotorcraft
    platform=-1,
    vib_thr=0.15,       # median accel white noise [m/s^2] above which the platform is a rotorcraft
    # features
    backward=1, adapt=1, repair=1, still=1, verbose=0,
)

# Motion priors learned per platform class on TRAIN flights (tools/bench/tune.py). Rotor vibration makes
# the class unambiguous: median accel noise 0.5-1.0 m/s^2 on EuRoC MAV, ~0.03 on handheld TUM-VI.
PRESETS = {
    1: dict(sig_v_h=4.0, sig_p_h=10.0, sig_a=0.4, ba_rw=0.04, ba_k=0.04, sig_c=0.05, tilt_sig=0.25, tilt_k=2.0,
            sig_gap=6.0, bg0_sig=0.005, blind_tau=0.0),  # rotorcraft (tuned on MH_01/03/05, V1_01, V2_02)
    0: dict(sig_v_h=1.96, sig_p_h=1.5, sig_a=0.05, ba_rw=0.02, ba_k=0.028, sig_c=0.1, tilt_sig=0.25,
            bg0_sig=0.005),  # handheld (tuned on room1/3/5, validated on room2/4/6; final refit on all six: sig_a 0.1 -> 0.05)
}
CLI_SET = set()  # parameters given on the command line override the presets


def log(*a):
    if P["verbose"]:
        print(*a, file=sys.stderr)


# ----------------------------------------------------------------------------- input handling
def resolve_input(arg):
    """Accept a CSV, a dataset folder, or a zip; returns an opener for a fresh text stream."""
    cands = ("imu0/data.csv", "mav0/imu0/data.csv", "data.csv")
    if arg is None:
        arg = "."
    if os.path.isdir(arg):
        for c in cands:
            p = os.path.join(arg, c)
            if os.path.isfile(p):
                return (lambda p=p: open(p, "r", newline=None, encoding="utf-8-sig", errors="replace")), p
        for root, _dirs, files in os.walk(arg):
            if "data.csv" in files and os.path.basename(root) == "imu0":
                p = os.path.join(root, "data.csv")
                return (lambda p=p: open(p, "r", newline=None, encoding="utf-8-sig", errors="replace")), p
        zips = [f for f in os.listdir(arg) if f.lower().endswith(".zip")]
        if len(zips) == 1:
            arg = os.path.join(arg, zips[0])
        else:
            raise SystemExit(f"no imu0/data.csv found under {arg}")
    if arg.lower().endswith(".zip"):
        import io
        import zipfile  # imported only for zip input (the module alone costs ~1 MB of heap)
        z = zipfile.ZipFile(arg)
        names = [n for n in z.namelist() if n.endswith("imu0/data.csv")] or \
                [n for n in z.namelist() if n.endswith(".csv")]
        if not names:
            raise SystemExit(f"no csv in {arg}")
        n = sorted(names, key=len)[0]
        return (lambda: io.TextIOWrapper(zipfile.ZipFile(arg).open(n), encoding="utf-8-sig", newline=None)), arg + ":" + n
    return (lambda: open(arg, "r", newline=None, encoding="utf-8-sig", errors="replace")), arg


def parse_ts(tok):
    """Timestamp token -> (int value or None, float value). Integers are kept exact."""
    try:
        return int(tok), None
    except ValueError:
        return None, float(tok)


# ----------------------------------------------------------------------------- pass 1a: format survey

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


def main(argv):
    if LOAD_ERROR is not None:
        print(f"pathguard: cannot load the estimator ({LOAD_ERROR}); writing a stand-still trajectory", file=sys.stderr)
        pos = [a for a in argv[1:] if not (a.startswith("--") and "=" in a)]
        fallback(pos[0] if pos else None, pos[1] if len(pos) > 1 else "estimated_trajectory.txt")
        return
    pos_args = []
    for a in argv[1:]:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            if k not in P:
                raise SystemExit(f"unknown parameter {k}")
            CLI_SET.add(k)
            P[k] = type(P[k])(float(v)) if not isinstance(P[k], str) else v
        else:
            pos_args.append(a)
    inp = pos_args[0] if pos_args else None
    out = pos_args[1] if len(pos_args) > 1 else "estimated_trajectory.txt"
    try:
        run(inp, out)
    except Exception as e:  # never leave the evaluator without a well-formed trajectory
        print(f"pathguard: estimation failed ({type(e).__name__}: {e}); writing a stand-still trajectory", file=sys.stderr)
        fallback(inp, out)


def fallback(inp, out):
    """One pose per input row (position 0, identity attitude): the output contract holds for any input."""
    opener, _ = resolve_input(inp)
    with opener() as f, open(out, "w", newline="\n") as fo:
        fo.write("# timestamp tx ty tz qx qy qz qw\n")
        for line in f:
            s = line.strip()
            if not s or not (s[0].isdigit() or s[0] in "+-."):
                continue
            tok = s.split(",", 1)[0].strip()
            try:
                ti, tf = parse_ts(tok)
            except ValueError:
                continue
            if ti is not None and ti >= 10**12:  # nanoseconds
                ds = str(ti)
                ts_s = ds[:-9] + "." + ds[-9:]
            else:
                ts_s = tok
            fo.write(ts_s + " 0.000000 0.000000 0.000000 0.000000000 0.000000000 0.000000000 1.000000000\n")


if __name__ == "__main__":
    main(sys.argv)
