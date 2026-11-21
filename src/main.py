#!/usr/bin/env python3
"""PathGuard: trust-aware inertial navigation under progressive sensor degradation.

Samsung PRISM XR Hackathon 2026, Track 03 (Path Finder).

    python main.py [imu0/data.csv | dataset folder | dataset .zip] [estimated_trajectory.txt] [--param=value ...]

Input : EuRoC-format IMU CSV (#timestamp[ns], w_x, w_y, w_z [rad/s], a_x, a_y, a_z [m/s^2]).
Output: TUM trajectory (timestamp[s] tx ty tz qx qy qz qw), exactly one pose per input row,
        including the rows of the 5 s after the IMU fails (dead-reckoned).

Pure Python standard library: no numpy, no compiled code. Memory grows with the number of
50 Hz bins and 10 Hz knots, never with the number of rows, and the file is streamed three times:

  Pass A  survey      format/columns/units, timestamps, failure point, per-second noise ("jamming meter")
  Pass B  integrity   reject frozen/duplicate/zero/NaN rows and spikes; preintegrate valid rows into bins
  Stage C attitude    error-state Kalman filter on bins, forward + backward, two-filter fusion
  Stage D position    per-axis Kalman filter + RTS smoother on 10 Hz knots with a motion prior;
                      the 5 s blind window is the same smoother with no data (prior-conditional forecast)
  Pass E  output      re-read the timestamps, interpolate pose per row, write
"""
import math
import os
import sys
from array import array

# ----------------------------------------------------------------------------- parameters
P = dict(
    bin_dt=0.02,        # attitude/preintegration bin [s]
    knot_dt=0.1,        # translation smoother step [s]
    # attitude
    tilt_sig=0.5,       # gravity-direction measurement noise per update [rad]
    tilt_k=4.0,         # extra tilt noise per | |f| - g | [rad per m/s^2]
    tilt_gate=3.0,      # skip gravity update if | |f| - g | exceeds this [m/s^2]
    tilt_every=2,       # gravity update every N bins
    gyro_floor=2e-3,    # minimum gyro white noise per sample [rad/s]
    bg_rw=2e-5,         # base gyro bias random walk [rad/s/sqrt(s)]
    bg_k=0.002,         # extra gyro bias RW per unit severity [rad/s/sqrt(s)] (script: 6e-4 aware, ~1e-3..1e-2 as-is)
    gap_gyro_k=10.0,    # gyro noise inflation inside gaps
    bg0_sig=0.05,       # initial gyro bias uncertainty [rad/s] (EuRoC raw bias reaches ~0.08)
    # translation (per-second parameterisation, independent of knot_dt)
    sig_a=0.4,          # accel noise per knot [m/s^2] (scaled by sqrt(noise ratio))
    sig_gap=3.0,        # accel uncertainty where there is no data [m/s^2]
    ba_rw=5e-4,         # base accel bias random walk [m/s^2/sqrt(s)]
    ba_k=0.01,          # extra accel bias RW per unit severity [m/s^2/sqrt(s)] (script: 0.01 aware)
    ba_kz=0.015,        # extra vertical bias RW per unit severity (1.5e-3 scale drift x g)
    sig_c=0.2,          # workspace-centre random walk [m/sqrt(s)]
    sig_v_h=0.3,        # speed prior, horizontal [m/s]
    sig_v_z=0.1,        # speed prior, vertical [m/s]
    sig_p_h=1.0,        # workspace prior, horizontal [m]
    sig_p_z=0.5,        # workspace prior, vertical [m]
    blind_tau=-1.0,     # blind forecast: <0 smoother prior, 0 hold, >0 decaying velocity time constant [s]
    # integrity
    spike_k=8.0,        # spike threshold in robust sigmas (must hold on all 6 channels)
    # still detection (zero-velocity / zero-angular-rate updates), 0.5 s windows
    still_g=0.04,       # max gyro std [rad/s]
    still_a=1.0,        # max accel std [m/s^2]
    still_dg=0.3,       # max | |mean f| - g | [m/s^2]
    zupt_sig=0.02,      # zero-velocity pseudo-measurement noise [m/s]
    zaru_sig=0.002,     # zero-angular-rate measurement noise floor [rad/s]
    still_tilt=0.02,    # gravity-direction noise while still [rad]
    # features
    backward=1, adapt=1, repair=1, still=1, verbose=0,
)


def log(*a):
    if P["verbose"]:
        print(*a, file=sys.stderr)


# ----------------------------------------------------------------------------- input handling
def resolve_input(arg):
    """Accept a CSV, a dataset folder, or a zip; returns an opener for a fresh text stream."""
    import io
    import zipfile
    cands = ("imu0/data.csv", "mav0/imu0/data.csv", "data.csv")
    if arg is None:
        arg = "."
    if os.path.isdir(arg):
        for c in cands:
            p = os.path.join(arg, c)
            if os.path.isfile(p):
                return (lambda p=p: open(p, "r", newline=None)), p
        for root, _dirs, files in os.walk(arg):
            if "data.csv" in files and os.path.basename(root) == "imu0":
                p = os.path.join(root, "data.csv")
                return (lambda p=p: open(p, "r", newline=None)), p
        zips = [f for f in os.listdir(arg) if f.lower().endswith(".zip")]
        if len(zips) == 1:
            arg = os.path.join(arg, zips[0])
        else:
            raise SystemExit(f"no imu0/data.csv found under {arg}")
    if arg.lower().endswith(".zip") and zipfile.is_zipfile(arg):
        z = zipfile.ZipFile(arg)
        names = [n for n in z.namelist() if n.endswith("imu0/data.csv")] or \
                [n for n in z.namelist() if n.endswith(".csv")]
        if not names:
            raise SystemExit(f"no csv in {arg}")
        n = sorted(names, key=len)[0]
        return (lambda: io.TextIOWrapper(zipfile.ZipFile(arg).open(n), encoding="utf-8-sig", newline=None)), arg + ":" + n
    return (lambda: open(arg, "r", newline=None, encoding="utf-8-sig")), arg


def parse_ts(tok):
    """Timestamp token -> (int value or None, float value). Integers are kept exact."""
    try:
        return int(tok), None
    except ValueError:
        return None, float(tok)


# ----------------------------------------------------------------------------- pass A: survey
def survey(opener):
    """Detect layout and units, count rows, locate failure, measure per-second noise."""
    header = None
    rows = 0
    first_ts = None
    first_tsf = None
    is_int = True
    dts = []
    prev = None
    last_ti, last_tf = None, None
    norms13, norms46 = [], []
    with opener() as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            c = s[0]
            if not (c.isdigit() or c in "+-."):
                if header is None and rows == 0:
                    header = s
                continue
            parts = s.replace(" ", "").replace("\t", "").split(",")
            try:
                ti, tf = parse_ts(parts[0])
            except ValueError:
                continue
            rows += 1
            if ti is None:
                is_int = False
            if rows == 1:
                first_ts, first_tsf = ti, tf
            last_ti, last_tf = ti, tf
            cur = ti if ti is not None else tf
            if prev is not None and rows <= 2001:
                d = cur - prev
                if d > 0:
                    dts.append(d)
            prev = cur
            if rows <= 4001 and len(parts) >= 7:
                try:
                    v = [float(x) for x in parts[1:7]]
                except ValueError:
                    continue
                if all(math.isfinite(x) for x in v) and any(v):
                    norms13.append(math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
                    norms46.append(math.sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]))
    if rows < 3:
        raise SystemExit("input has fewer than 3 IMU rows")
    # --- timestamp unit -> seconds scale (same rule as the official degradation script)
    dts.sort()
    md = dts[len(dts) // 2] if dts else 1
    if md < 1.0:
        tscale = 1.0
    elif md < 1e3:
        tscale = 1e-3
    elif md < 1e6:
        tscale = 1e-6
    else:
        tscale = 1e-9
    dt_nom = md * tscale if dts else 0.005
    # --- column layout: header names first, magnitudes as fallback
    gyro_first = True
    if header:
        h = [x.strip().lower() for x in header.lstrip("#").split(",")]
        if len(h) >= 7:
            def kind(name):
                if name.startswith("w") or "gyr" in name or "omega" in name or "rad" in name:
                    return "g"
                if name.startswith("a") or "acc" in name or "m s^-2" in name or "m/s" in name:
                    return "a"
                return "?"
            k1, k4 = kind(h[1]), kind(h[4])
            if k1 == "a" and k4 == "g":
                gyro_first = False
            elif k1 == "?" or k4 == "?":
                header = None  # unknown names: fall back to magnitudes
    med = lambda x: sorted(x)[len(x) // 2] if x else 0.0
    m13, m46 = med(norms13), med(norms46)
    if not header and norms13:
        # accelerometer triple carries gravity (|f| ~ 9.8 m/s^2 or ~1 g)
        def gscore(m):
            return min(abs(m - 9.80665), abs(m - 1.0) * 9.80665)
        gyro_first = gscore(m46) <= gscore(m13)
    ma = m46 if gyro_first else m13
    ascale = 9.80665 if 0.5 < ma < 2.0 else 1.0  # accel given in g
    if last_ti is not None and first_ts is not None:
        t_end = (last_ti - first_ts) * tscale
    else:
        t_end = ((last_ti if last_ti is not None else last_tf) - (first_ts if first_ts is not None else first_tsf)) * tscale
    return dict(rows=rows, header=header, tscale=tscale, dt_nom=dt_nom, gyro_first=gyro_first,
                ascale=ascale, is_int=is_int, first_ts=first_ts, first_tsf=first_tsf, t_end=t_end)


def row_reader(opener, info):
    """Yields (t [s from first row], gx, gy, gz, ax, ay, az, valid_numeric) for every data row."""
    t0i, t0f, ts = info["first_ts"], info["first_tsf"], info["tscale"]
    gf, asc = info["gyro_first"], info["ascale"]
    isfin = math.isfinite
    with opener() as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            c = s[0]
            if not (c.isdigit() or c in "+-."):
                continue
            parts = s.replace(" ", "").replace("\t", "").split(",")
            try:
                ti, tf = parse_ts(parts[0])
            except ValueError:
                continue
            if ti is not None and t0i is not None:
                t = (ti - t0i) * ts
            else:
                t = ((ti if ti is not None else tf) - (t0i if t0i is not None else t0f)) * ts
            ok = len(parts) >= 7
            if ok:
                try:
                    v = [float(x) for x in parts[1:7]]
                except ValueError:
                    ok = False
            if ok:
                ok = isfin(v[0]) and isfin(v[1]) and isfin(v[2]) and isfin(v[3]) and isfin(v[4]) and isfin(v[5])
            if not ok:
                yield t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False
                continue
            if gf:
                yield t, v[0], v[1], v[2], v[3] * asc, v[4] * asc, v[5] * asc, True
            else:
                yield t, v[3], v[4], v[5], v[0] * asc, v[1] * asc, v[2] * asc, True


# ----------------------------------------------------------------------------- jamming meter
def noise_meter(opener, info):
    """Per-second statistics in one streaming pass:
      * robust white-noise level of gyro and accel (MAD of second differences of valid rows)
      * unique timestamps present (-> missing rows = packet loss)
    and the time of the last valid row (IMU failure)."""
    dt_nom = info["dt_nom"]
    sg, sa = array("f"), array("f")
    present = array("H")
    bg, ba = [], []
    w_idx = 0
    npres = 0
    p1 = p2 = None  # previous two valid rows (t, 6 values)
    t_last_valid = 0.0
    last = None
    last_t = None
    n_valid = 0
    for r in row_reader(opener, info):
        t = r[0]
        while t >= w_idx + 1.0:
            sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba)); present.append(min(npres, 65535))
            bg, ba = [], []; npres = 0; w_idx += 1
        if t != last_t:
            npres += 1
            last_t = t
        if not r[7]:
            continue
        vals = r[1:7]
        if not any(vals) or vals == last:  # dead (all zero) or frozen/duplicate row
            last = vals
            continue
        last = vals
        n_valid += 1
        t_last_valid = t
        if p1 is not None and p2 is not None:
            d1, d2 = t - p1[0], p1[0] - p2[0]
            if 0.5 * dt_nom < d1 < 1.5 * dt_nom and 0.5 * dt_nom < d2 < 1.5 * dt_nom:
                a, b, c = vals, p1[1], p2[1]
                bg.append(abs(a[0] - 2 * b[0] + c[0])); bg.append(abs(a[1] - 2 * b[1] + c[1])); bg.append(abs(a[2] - 2 * b[2] + c[2]))
                ba.append(abs(a[3] - 2 * b[3] + c[3])); ba.append(abs(a[4] - 2 * b[4] + c[4])); ba.append(abs(a[5] - 2 * b[5] + c[5]))
        p2, p1 = p1, (t, vals)
    sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba)); present.append(min(npres, 65535))
    return sg, sa, present, t_last_valid, n_valid


def _mad_sigma(x):
    if len(x) < 30:
        return -1.0
    x.sort()
    return 1.4826 * x[len(x) // 2] / math.sqrt(6.0)


def smooth_levels(sig, T_L):
    """Fill missing windows and apply a 5-window running median (robust per-second noise level)."""
    n = len(sig)
    good = [x for i, x in enumerate(sig) if x > 0 and i <= T_L]
    fill = sorted(good)[len(good) // 2] if good else 0.0
    v = [x if x > 0 else -1.0 for x in sig]
    out = array("f", bytes(4 * n))
    for i in range(n):
        w = [x for x in v[max(0, i - 2):i + 3] if x > 0]
        out[i] = sorted(w)[len(w) // 2] if w else fill
    return out


def fit_severity(present, dt_nom, T_L, T_end):
    """Jamming level from packet loss. The official degradation drops rows with probability
    0.06 s per row plus bursts (start prob 0.0015 s, length U[5, 5 + 45 s]); s(t) = L (t/T)^gamma is the
    single severity that also scales noise, bias drift, scale drift, spikes and frozen samples.
    Fit (L, gamma) by least squares on the per-second missing-row counts (failure window excluded)."""
    exp_per_win = 1.0 / dt_nom
    pts = []
    for i, n in enumerate(present):
        if i + 1.0 > T_L or i < 2:  # skip the failure window, the last partial second and the protected start
            continue
        pts.append(((i + 0.5) / max(T_end, 1e-9), max(0.0, exp_per_win - n) / exp_per_win))
    if len(pts) < 5:
        return 0.0, 1.7

    def miss(sv):
        return 0.06 * sv + 0.0015 * sv * (5.0 + 22.5 * sv)
    best = (1e30, 0.0, 1.7)

    def fit_L(g):
        xs = [r ** g for r, _ in pts]
        ys = [y for _, y in pts]
        lo, hi = 0.0, 3.0
        for _ in range(28):  # ternary search on L (convex for fixed gamma)
            m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            e1 = e2 = 0.0
            for x, y in zip(xs, ys):
                u1 = m1 * x; u2 = m2 * x
                d1 = 0.0675 * u1 + 0.03375 * u1 * u1 - y; d2 = 0.0675 * u2 + 0.03375 * u2 * u2 - y
                e1 += d1 * d1; e2 += d2 * d2
            if e1 < e2:
                hi = m2
            else:
                lo = m1
        Lh = 0.5 * (lo + hi)
        e = 0.0
        for x, y in zip(xs, ys):
            u = Lh * x; d = 0.0675 * u + 0.03375 * u * u - y; e += d * d
        return e, Lh
    for g in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
        e, Lh = fit_L(g)
        if e < best[0]:
            best = (e, Lh, g)
    g0 = best[2]
    for g in (g0 - 0.375, g0 - 0.25, g0 - 0.125, g0 + 0.125, g0 + 0.25, g0 + 0.375):
        if g <= 0.25:
            continue
        e, Lh = fit_L(g)
        if e < best[0]:
            best = (e, Lh, g)
    return best[1], best[2]


# ----------------------------------------------------------------------------- pass B: integrity + bins
def preintegrate(opener, info, sg, sa, T_L):
    """Valid-row stream -> spike rejection (5-row window) -> per-bin mean gyro and accel."""
    bdt = P["bin_dt"]
    nb = int(T_L / bdt) + 1
    gsum = array("d", bytes(8 * 3 * nb))
    asum = array("d", bytes(8 * 3 * nb))
    cnt = array("H", bytes(2 * nb))
    nw = len(sg)
    kk = P["spike_k"]
    repair = P["repair"]
    buf = []  # last 5 valid rows
    last = None
    stats = dict(dead=0, frozen=0, spikes=0, valid=0, nan=0)
    nwin = int(T_L / 0.5) + 1
    ws = array("d", bytes(8 * 13 * nwin))  # per 0.5 s window: n, sum w(3), sum w^2(3), sum f(3), sum f^2(3)

    def emit(r):
        t = r[0]
        if t > T_L + 1e-9:
            return
        b = int(t / bdt)
        if b >= nb:
            b = nb - 1
        o = 13 * min(nwin - 1, int(t * 2.0))
        ws[o] += 1.0
        ws[o + 1] += r[1]; ws[o + 2] += r[2]; ws[o + 3] += r[3]
        ws[o + 4] += r[1] * r[1]; ws[o + 5] += r[2] * r[2]; ws[o + 6] += r[3] * r[3]
        ws[o + 7] += r[4]; ws[o + 8] += r[5]; ws[o + 9] += r[6]
        ws[o + 10] += r[4] * r[4]; ws[o + 11] += r[5] * r[5]; ws[o + 12] += r[6] * r[6]
        i3 = 3 * b
        gsum[i3] += r[1]; gsum[i3 + 1] += r[2]; gsum[i3 + 2] += r[3]
        asum[i3] += r[4]; asum[i3 + 1] += r[5]; asum[i3 + 2] += r[6]
        if cnt[b] < 65535:
            cnt[b] += 1
        stats["valid"] += 1

    def check(i):
        """Spike signature of the jamming model: the SAME sample jumps on all six channels
        (every gyro and accel axis) at once. Real motion, even fast handheld rotation, does not.
        A channel 'jumps' if it is more than spike_k robust sigmas from the median of its
        two neighbours on each side. Measured on labelled official degradations:
        precision 0.97, recall 0.85 (a per-channel rule gives precision 0.16)."""
        r = buf[i]
        w = min(nw - 1, int(r[0]))
        s_g = kk * (sg[w] if sg[w] > 0 else 0.01)
        s_a = kk * (sa[w] if sa[w] > 0 else 0.1)
        o = [buf[j] for j in range(5) if j != i]
        for c in range(1, 7):
            v = sorted((o[0][c], o[1][c], o[2][c], o[3][c]))
            if abs(r[c] - 0.5 * (v[1] + v[2])) <= (s_g if c <= 3 else s_a):
                return False
        return True

    for r in row_reader(opener, info):
        if not r[7]:
            stats["nan"] += 1
            continue
        vals = r[1:7]
        if not any(vals):
            stats["dead"] += 1
            last = vals
            continue
        if vals == last:
            stats["frozen"] += 1
            continue
        last = vals
        if r[0] > T_L + 1e-9:
            continue
        if not repair:
            emit(r)
            continue
        buf.append(r)
        if len(buf) == 5:
            if check(2):
                stats["spikes"] += 1
                del buf[2]
                continue
            if len(buf) == 5:
                emit(buf.pop(0))
                # buf now holds 4; the next row completes the window around buf[2]
    # flush: the last <=4 rows before the failure have no two-sided window; keep them
    for r in buf:
        emit(r)
    return gsum, asum, cnt, nb, stats, ws, nwin


def still_windows(ws, nwin, gmag):
    """0.5 s windows in which the body is at rest: low gyro and accel spread and |f| ~ g.
    Chosen on ground-truth-labelled windows for ~zero false positives (a false zero-velocity
    update in flight would hurt far more than a missed one). A window counts only if a
    neighbour is also still (>= 1 s of rest)."""
    dt_nom = INFO["dt_nom"]
    raw = array("b", bytes(nwin))
    for k in range(nwin):
        o = 13 * k
        n = ws[o]
        if n < 0.6 * 0.5 / dt_nom:
            continue
        vg = va = 0.0
        for c in range(3):
            m = ws[o + 1 + c] / n
            vg += max(0.0, ws[o + 4 + c] / n - m * m)
            m = ws[o + 7 + c] / n
            va += max(0.0, ws[o + 10 + c] / n - m * m)
        fm = math.sqrt(sum((ws[o + 7 + c] / n) ** 2 for c in range(3)))
        if math.sqrt(vg) < P["still_g"] and math.sqrt(va) < P["still_a"] and abs(fm - gmag) < P["still_dg"]:
            raw[k] = 1
    out = array("b", bytes(nwin))
    for k in range(nwin):
        if raw[k] and ((k > 0 and raw[k - 1]) or (k + 1 < nwin and raw[k + 1])):
            out[k] = 1
    return out


# ----------------------------------------------------------------------------- rotation helpers
def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def qexp(x, y, z):
    a = math.sqrt(x * x + y * y + z * z)
    if a < 1e-12:
        return (1.0, 0.5 * x, 0.5 * y, 0.5 * z)
    s = math.sin(0.5 * a) / a
    return (math.cos(0.5 * a), s * x, s * y, s * z)


def qnorm(q):
    n = 1.0 / math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] * n, q[1] * n, q[2] * n, q[3] * n)


def qlog(q):
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    vn = math.sqrt(x * x + y * y + z * z)
    if vn < 1e-12:
        return (2 * x, 2 * y, 2 * z)
    a = 2 * math.atan2(vn, w) / vn
    return (a * x, a * y, a * z)


def qrot(q, v):
    """R(q) v, q body->world."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy); ty = 2 * (z * vx - x * vz); tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


def qmat(q):
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


# 3x3 helpers on tuples-of-tuples (row major)
def m3mul(A, B):
    return tuple(tuple(A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j] for j in range(3)) for i in range(3))


def m3T(A):
    return ((A[0][0], A[1][0], A[2][0]), (A[0][1], A[1][1], A[2][1]), (A[0][2], A[1][2], A[2][2]))


def m3add(A, B, s=1.0):
    return tuple(tuple(A[i][j] + s * B[i][j] for j in range(3)) for i in range(3))


def m3inv(A):
    a, b, c = A[0]; d, e, f = A[1]; g, h, i = A[2]
    co0, co1, co2 = e * i - f * h, f * g - d * i, d * h - e * g
    det = a * co0 + b * co1 + c * co2
    if abs(det) < 1e-300:
        return None
    r = 1.0 / det
    return ((co0 * r, (c * h - b * i) * r, (b * f - c * e) * r),
            (co1 * r, (a * i - c * g) * r, (c * d - a * f) * r),
            (co2 * r, (b * g - a * h) * r, (a * e - b * d) * r))


def m3sym(A):
    return tuple(tuple(0.5 * (A[i][j] + A[j][i]) for j in range(3)) for i in range(3))


I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
Z3 = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def diag3(a):
    return ((a, 0.0, 0.0), (0.0, a, 0.0), (0.0, 0.0, a))


# ----------------------------------------------------------------------------- stage C: attitude
class Att:
    """ESKF on [dtheta (world frame), dbg (body)]. R_true = Exp(dtheta) R_hat.

    In the world frame the gravity measurement only sees roll/pitch (dtheta_x, dtheta_y),
    so the update is a 2-D measurement; heading is propagated by the gyro only.
    Covariance blocks: A = cov(dtheta), B = cov(dtheta, dbg), C = cov(dbg)."""
    __slots__ = ("q", "bg", "A", "B", "C")

    def __init__(self, q, bg, sth, syaw, sbg):
        self.q = q; self.bg = bg
        self.A = ((sth * sth, 0.0, 0.0), (0.0, sth * sth, 0.0), (0.0, 0.0, syaw * syaw))
        self.B = Z3
        self.C = diag3(sbg * sbg)

    def predict(self, w, dt, qg2, qb2):
        """Propagate over dt (negative = backward) with body rate w. qg2: rotation noise var added,
        qb2: bias RW var added.  Error model: dtheta' = dtheta - dt R dbg  (F = [[I, -dt R], [0, I]]):
            A' = A - dt (R B' + B R') + dt^2 R C R' + qg2 I,   B' = B - dt R C,   C' = C + qb2 I."""
        bg = self.bg
        q = self.q
        qw, qx, qy, qz = q
        self.q = qnorm(qmul(q, qexp((w[0] - bg[0]) * dt, (w[1] - bg[1]) * dt, (w[2] - bg[2]) * dt)))
        # rotation matrix of the pre-step attitude
        r00 = 1 - 2 * (qy * qy + qz * qz); r01 = 2 * (qx * qy - qw * qz); r02 = 2 * (qx * qz + qw * qy)
        r10 = 2 * (qx * qy + qw * qz); r11 = 1 - 2 * (qx * qx + qz * qz); r12 = 2 * (qy * qz - qw * qx)
        r20 = 2 * (qx * qz - qw * qy); r21 = 2 * (qy * qz + qw * qx); r22 = 1 - 2 * (qx * qx + qy * qy)
        (a00, a01, a02), (_, a11, a12), (_, _, a22) = self.A
        (b00, b01, b02), (b10, b11, b12), (b20, b21, b22) = self.B
        (c00, c01, c02), (_, c11, c12), (_, _, c22) = self.C
        # RC = R C
        e00 = r00 * c00 + r01 * c01 + r02 * c02; e01 = r00 * c01 + r01 * c11 + r02 * c12; e02 = r00 * c02 + r01 * c12 + r02 * c22
        e10 = r10 * c00 + r11 * c01 + r12 * c02; e11 = r10 * c01 + r11 * c11 + r12 * c12; e12 = r10 * c02 + r11 * c12 + r12 * c22
        e20 = r20 * c00 + r21 * c01 + r22 * c02; e21 = r20 * c01 + r21 * c11 + r22 * c12; e22 = r20 * c02 + r21 * c12 + r22 * c22
        # M = R B'  (M_ij = sum_k R_ik B_jk)
        m00 = r00 * b00 + r01 * b01 + r02 * b02; m01 = r00 * b10 + r01 * b11 + r02 * b12; m02 = r00 * b20 + r01 * b21 + r02 * b22
        m10 = r10 * b00 + r11 * b01 + r12 * b02; m11 = r10 * b10 + r11 * b11 + r12 * b12; m12 = r10 * b20 + r11 * b21 + r12 * b22
        m20 = r20 * b00 + r21 * b01 + r22 * b02; m21 = r20 * b10 + r21 * b11 + r22 * b12; m22 = r20 * b20 + r21 * b21 + r22 * b22
        # S = R C R'
        s00 = e00 * r00 + e01 * r01 + e02 * r02; s01 = e00 * r10 + e01 * r11 + e02 * r12; s02 = e00 * r20 + e01 * r21 + e02 * r22
        s11 = e10 * r10 + e11 * r11 + e12 * r12; s12 = e10 * r20 + e11 * r21 + e12 * r22
        s22 = e20 * r20 + e21 * r21 + e22 * r22
        d2 = dt * dt
        n00 = a00 - 2 * dt * m00 + d2 * s00 + qg2
        n01 = a01 - dt * (m01 + m10) + d2 * s01
        n02 = a02 - dt * (m02 + m20) + d2 * s02
        n11 = a11 - 2 * dt * m11 + d2 * s11 + qg2
        n12 = a12 - dt * (m12 + m21) + d2 * s12
        n22 = a22 - 2 * dt * m22 + d2 * s22 + qg2
        self.A = ((n00, n01, n02), (n01, n11, n12), (n02, n12, n22))
        self.B = ((b00 - dt * e00, b01 - dt * e01, b02 - dt * e02),
                  (b10 - dt * e10, b11 - dt * e11, b12 - dt * e12),
                  (b20 - dt * e20, b21 - dt * e21, b22 - dt * e22))
        self.C = ((c00 + qb2, c01, c02), (c01, c11 + qb2, c12), (c02, c12, c22 + qb2))

    def zero_rate(self, w, sig):
        """At rest the true rate is zero: z = w - bg ~ N(0, sig^2) measures the gyro bias on all three axes
        (including the one about gravity, which tilt updates cannot see). H = [0 I]."""
        A, B, C = self.A, self.B, self.C
        z = (w[0] - self.bg[0], w[1] - self.bg[1], w[2] - self.bg[2])
        S = m3add(C, diag3(sig * sig))
        Si = m3inv(S)
        if Si is None:
            return
        KA = m3mul(B, Si)          # gain for dtheta  (3x3) = B S^-1
        KC = m3mul(C, Si)          # gain for dbg     (3x3) = C S^-1
        dth = tuple(KA[i][0] * z[0] + KA[i][1] * z[1] + KA[i][2] * z[2] for i in range(3))
        dbg = tuple(KC[i][0] * z[0] + KC[i][1] * z[1] + KC[i][2] * z[2] for i in range(3))
        self.q = qnorm(qmul(qexp(dth[0], dth[1], dth[2]), self.q))
        self.bg = (self.bg[0] + dbg[0], self.bg[1] + dbg[1], self.bg[2] + dbg[2])
        Bt = m3T(B)
        # P <- P - K [B' C]:  A -= KA B',  B -= KA C,  C -= KC C
        self.A = m3sym(m3add(A, m3mul(KA, Bt), -1.0))
        self.B = m3add(B, m3mul(KA, C), -1.0)
        self.C = m3sym(m3add(C, m3mul(KC, C), -1.0))

    def gravity(self, f, sig):
        """f: mean body specific force. Measurement z = (R f / |f|)_xy  ~ (-dth_y, dth_x)."""
        n = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
        if n < 1e-6:
            return
        wx, wy, _wz = qrot(self.q, (f[0] / n, f[1] / n, f[2] / n))
        A, B = self.A, self.B
        # H selects (-A row y, A row x): z1 = -dth_y, z2 = dth_x
        # P H^T columns: for theta block  col1 = -A[:,1], col2 = A[:,0]; for bias block col1 = -B^T[:,1], col2 = B^T[:,0]
        s11 = A[1][1] + sig * sig
        s12 = -A[1][0]
        s22 = A[0][0] + sig * sig
        det = s11 * s22 - s12 * s12
        if det <= 0:
            return
        i11, i12, i22 = s22 / det, -s12 / det, s11 / det
        # PHt rows (6 x 2)
        # cov(dbg_r, dtheta_c) = B[c][r]
        PH = [(-A[r][1], A[r][0]) for r in range(3)] + [(-B[1][r], B[0][r]) for r in range(3)]
        K = [(ph[0] * i11 + ph[1] * i12, ph[0] * i12 + ph[1] * i22) for ph in PH]
        z1, z2 = wx, wy
        dx = [k[0] * z1 + k[1] * z2 for k in K]
        # apply: R <- Exp(dth) R ; bg <- bg + dbg
        self.q = qnorm(qmul(qexp(dx[0], dx[1], dx[2]), self.q))
        self.bg = (self.bg[0] + dx[3], self.bg[1] + dx[4], self.bg[2] + dx[5])
        # P <- P - K (H P) ; (H P) rows: row1 = -P[1,:], row2 = P[0,:]   (P = [[A, B],[B^T, C]])
        Prow0 = (A[0][0], A[0][1], A[0][2], B[0][0], B[0][1], B[0][2])
        Prow1 = (A[1][0], A[1][1], A[1][2], B[1][0], B[1][1], B[1][2])
        HP0 = tuple(-x for x in Prow1)
        HP1 = Prow0
        full = [[0.0] * 6 for _ in range(6)]
        Pm = [list(A[0]) + list(B[0]), list(A[1]) + list(B[1]), list(A[2]) + list(B[2]),
              [B[0][0], B[1][0], B[2][0]] + list(self.C[0]),
              [B[0][1], B[1][1], B[2][1]] + list(self.C[1]),
              [B[0][2], B[1][2], B[2][2]] + list(self.C[2])]
        for r in range(6):
            kr0, kr1 = K[r]
            row = Pm[r]
            full[r] = [row[c] - kr0 * HP0[c] - kr1 * HP1[c] for c in range(6)]
        for r in range(6):
            for c in range(r + 1, 6):
                m = 0.5 * (full[r][c] + full[c][r]); full[r][c] = full[c][r] = m
        self.A = tuple(tuple(full[r][0:3]) for r in range(3))
        self.B = tuple(tuple(full[r][3:6]) for r in range(3))
        self.C = tuple(tuple(full[r][3:6]) for r in range(3, 6))


def initial_attitude(gsum, asum, cnt, nb, sg):
    """Gravity direction (roll, pitch; yaw = 0) and gyro bias from the calmest early second."""
    bdt = P["bin_dt"]
    per = max(1, int(round(1.0 / bdt)))
    lim = min(nb, int(60.0 / bdt))
    best, bestv = 0, 1e30
    for s in range(0, max(1, lim - per + 1), max(1, per // 4)):
        n = 0; m = [0.0] * 3; m2 = [0.0] * 3
        for b in range(s, min(nb, s + per)):
            c = cnt[b]
            if not c:
                continue
            for k in range(3):
                w = gsum[3 * b + k] / c
                m[k] += w; m2[k] += w * w
            n += 1
        if n < per // 2:
            continue
        v = sum(m2[k] / n - (m[k] / n) ** 2 for k in range(3))
        if v < bestv:
            bestv, best = v, s
    am = [0.0] * 3; wm = [0.0] * 3; n = 0
    for b in range(best, min(nb, best + per)):
        c = cnt[b]
        if c:
            for k in range(3):
                am[k] += asum[3 * b + k]; wm[k] += gsum[3 * b + k]
            n += c
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 9.81, best, bestv
    am = [x / n for x in am]; wm = [x / n for x in wm]
    g = math.sqrt(sum(x * x for x in am))
    still = bestv < 2.5e-3 * 0.25 and abs(g - 9.81) < 0.5  # bin-mean variance is ~4x lower than per-sample
    bg0 = tuple(wm) if still else (0.0, 0.0, 0.0)
    u = [x / g for x in am]
    # rotation taking body "up" u to world z
    ax, ay, az = u[1] * 1.0 - u[2] * 0.0, u[2] * 0.0 - u[0] * 1.0, 0.0
    ax, ay, az = u[1], -u[0], 0.0
    s = math.sqrt(ax * ax + ay * ay)
    cth = u[2]
    if s < 1e-9:
        q0 = (1.0, 0.0, 0.0, 0.0) if cth > 0 else (0.0, 1.0, 0.0, 0.0)
    else:
        ang = math.atan2(s, cth)
        q0 = qexp(ax / s * ang, ay / s * ang, 0.0)
    # roll back from the calm window to t = 0 with the gyro
    for b in range(best - 1, -1, -1):
        c = cnt[b]
        if c:
            w = [gsum[3 * b + k] / c - bg0[k] for k in range(3)]
            q0 = qnorm(qmul(q0, qexp(-w[0] * bdt, -w[1] * bdt, -w[2] * bdt)))
    return q0, bg0, g, best, bestv


def attitude(gsum, asum, cnt, nb, T_L, gmag):
    """Forward + backward ESKF over bins; returns smoothed quaternion at every bin boundary (nb+1)."""
    bdt = P["bin_dt"]
    every = max(1, int(P["tilt_every"]))
    dt_nom = INFO["dt_nom"]
    adapt = P["adapt"]
    n_tilt = max(1.0, every * bdt / dt_nom)  # samples averaged per gravity update

    # gyro for empty bins: linear interpolation between neighbouring filled bins (filled in place)
    filled = array("b", bytes(nb))
    for b in range(nb):
        filled[b] = 1 if cnt[b] else 0
    b = 0
    while b < nb:
        if filled[b]:
            b += 1; continue
        a0 = b
        while b < nb and not filled[b]:
            b += 1
        left = a0 - 1 if a0 > 0 else (b if b < nb else None)
        right = b if b < nb else left
        if left is None:
            break
        wl = [gsum[3 * left + k] / cnt[left] for k in range(3)]
        wr = [gsum[3 * right + k] / cnt[right] for k in range(3)]
        span = right - left if right != left else 1
        for j in range(a0, b):
            s = (j - left) / span if right != left else 0.0
            for k in range(3):
                gsum[3 * j + k] = (1 - s) * wl[k] + s * wr[k]
            # cnt stays 0: marks interpolated

    def gyro_var(b):
        """Rotation noise variance over one bin: measured white gyro noise integrated over the bin."""
        sg_ = max(sigma_g(b * bdt), P["gyro_floor"]) if adapt else P["gyro_floor"]
        v = sg_ * sg_ * dt_nom * bdt
        if not cnt[b]:
            v *= P["gap_gyro_k"] ** 2
        return v

    def bias_var(b):
        q = P["bg_rw"] + (P["bg_k"] * sev_at(b * bdt) if adapt else 0.0)
        return q * q * bdt

    def is_still(b):
        if STILL is None:
            return False
        k = int(b * bdt * 2.0)
        return 0 <= k < len(STILL) and STILL[k] == 1

    def tilt_sig(f, b):
        n = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
        dev = abs(n - gmag)
        if is_still(b):
            return P["still_tilt"]
        if dev > P["tilt_gate"]:
            return None
        s2 = P["tilt_sig"] ** 2 + (P["tilt_k"] * dev) ** 2
        if adapt:
            sa_ = sigma_a(b * bdt)
            s2 += sa_ * sa_ / (n_tilt * gmag * gmag)
        return math.sqrt(s2)

    q0, bg0, _g, _best, _v = INIT

    def rate(b):
        c = cnt[b]
        if c:
            return (gsum[3 * b] / c, gsum[3 * b + 1] / c, gsum[3 * b + 2] / c)
        return (gsum[3 * b], gsum[3 * b + 1], gsum[3 * b + 2])  # interpolated (empty bin)

    # ---- forward ESKF; store posterior nominal state and covariance at every bin boundary
    F = Att(q0, bg0, 0.02, 0.02, P["bg0_sig"])
    nq = nb + 1
    qf = array("f", bytes(4 * 4 * nq))      # quaternion (w, x, y, z)
    bgf = array("f", bytes(4 * 3 * nq))     # gyro bias
    Pf = array("f", bytes(4 * 21 * nq))     # upper triangle of the 6x6 covariance

    def store(i, Fi):
        q = Fi.q
        qf[4 * i] = q[0]; qf[4 * i + 1] = q[1]; qf[4 * i + 2] = q[2]; qf[4 * i + 3] = q[3]
        bgf[3 * i] = Fi.bg[0]; bgf[3 * i + 1] = Fi.bg[1]; bgf[3 * i + 2] = Fi.bg[2]
        A, B, C = Fi.A, Fi.B, Fi.C
        Pf[21 * i:21 * i + 21] = array("f", (A[0][0], A[0][1], A[0][2], B[0][0], B[0][1], B[0][2],
                                             A[1][1], A[1][2], B[1][0], B[1][1], B[1][2],
                                             A[2][2], B[2][0], B[2][1], B[2][2],
                                             C[0][0], C[0][1], C[0][2], C[1][1], C[1][2], C[2][2]))

    acc = [0.0, 0.0, 0.0]; na = 0; since = 0
    store(0, F)
    for b in range(nb):
        F.predict(rate(b), bdt, gyro_var(b), bias_var(b))
        c = cnt[b]
        if c:
            acc[0] += asum[3 * b] / c; acc[1] += asum[3 * b + 1] / c; acc[2] += asum[3 * b + 2] / c; na += 1
        since += 1
        if since >= every and na:
            f = (acc[0] / na, acc[1] / na, acc[2] / na)
            s = tilt_sig(f, b)
            if s is not None:
                F.gravity(f, s)
            acc = [0.0, 0.0, 0.0]; na = 0; since = 0
        if c and is_still(b):
            sz = max(P["zaru_sig"], sigma_g(b * bdt) / math.sqrt(c))
            F.zero_rate(rate(b), sz)
        store(b + 1, F)
    bg_end = F.bg
    if not P["backward"]:
        return qf, bg_end

    # ---- error-state RTS smoother (backward). For each step k -> k+1:
    #   x_p(k+1) = f(x_f(k)),  P_p = F P_f F' + Q,   delta = P_f F' P_p^-1 (x_s(k+1) [-] x_p(k+1)),
    #   x_s(k) = x_f(k) [+] delta.  The smoothed gyro bias carries information from the whole flight
    #   back to t = 0, which makes heading consistent (the bias about the vertical is only weakly
    #   observable, through the times the body tilts).
    qs = (qf[4 * nb], qf[4 * nb + 1], qf[4 * nb + 2], qf[4 * nb + 3])
    bs = (bgf[3 * nb], bgf[3 * nb + 1], bgf[3 * nb + 2])
    out = qf  # overwritten in place from the end backwards (entry k+1 is no longer needed as filtered)
    tmp = Att((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
    for k in range(nb - 1, -1, -1):
        qk = (qf[4 * k], qf[4 * k + 1], qf[4 * k + 2], qf[4 * k + 3])
        bk = (bgf[3 * k], bgf[3 * k + 1], bgf[3 * k + 2])
        p = Pf[21 * k:21 * k + 21]
        A = ((p[0], p[1], p[2]), (p[1], p[6], p[7]), (p[2], p[7], p[11]))
        B = ((p[3], p[4], p[5]), (p[8], p[9], p[10]), (p[12], p[13], p[14]))
        C = ((p[15], p[16], p[17]), (p[16], p[18], p[19]), (p[17], p[19], p[20]))
        tmp.q, tmp.bg, tmp.A, tmp.B, tmp.C = qk, bk, A, B, C
        tmp.predict(rate(k), bdt, gyro_var(k), bias_var(k))
        # difference x_s(k+1) [-] x_p(k+1) in the world-frame error convention
        qp = tmp.q
        dth = qlog(qmul(qs, (qp[0], -qp[1], -qp[2], -qp[3])))
        d = [dth[0], dth[1], dth[2], bs[0] - tmp.bg[0], bs[1] - tmp.bg[1], bs[2] - tmp.bg[2]]
        y = chol_solve6(tmp.A, tmp.B, tmp.C, d)
        if y is None:
            qs, bs = qk, bk
        else:
            # F' y = (y_th, -dt R' y_th + y_b), R = R(q_f(k))
            R = qmat(qk)
            yt = y[0:3]
            z = (y[0], y[1], y[2],
                 y[3] - bdt * (R[0][0] * yt[0] + R[1][0] * yt[1] + R[2][0] * yt[2]),
                 y[4] - bdt * (R[0][1] * yt[0] + R[1][1] * yt[1] + R[2][1] * yt[2]),
                 y[5] - bdt * (R[0][2] * yt[0] + R[1][2] * yt[1] + R[2][2] * yt[2]))
            M = (A[0] + B[0], A[1] + B[1], A[2] + B[2],
                 (B[0][0], B[1][0], B[2][0]) + C[0], (B[0][1], B[1][1], B[2][1]) + C[1], (B[0][2], B[1][2], B[2][2]) + C[2])
            dl = [M[r][0] * z[0] + M[r][1] * z[1] + M[r][2] * z[2] + M[r][3] * z[3] + M[r][4] * z[4] + M[r][5] * z[5]
                  for r in range(6)]
            qs = qnorm(qmul(qexp(dl[0], dl[1], dl[2]), qk))
            bs = (bk[0] + dl[3], bk[1] + dl[4], bk[2] + dl[5])
        out[4 * k] = qs[0]; out[4 * k + 1] = qs[1]; out[4 * k + 2] = qs[2]; out[4 * k + 3] = qs[3]
    return out, bg_end


def chol_solve6(A, B, C, d):
    """Solve P y = d for the symmetric positive-definite 6x6 P = [[A, B], [B', C]] (Cholesky)."""
    Pm = (A[0] + B[0], A[1] + B[1], A[2] + B[2],
          (B[0][0], B[1][0], B[2][0]) + C[0], (B[0][1], B[1][1], B[2][1]) + C[1], (B[0][2], B[1][2], B[2][2]) + C[2])
    L = [[0.0] * 6 for _ in range(6)]
    for i in range(6):
        Li = L[i]
        for j in range(i + 1):
            Lj = L[j]
            s = Pm[i][j]
            for k in range(j):
                s -= Li[k] * Lj[k]
            if i == j:
                if s <= 0.0:
                    return None
                Li[i] = math.sqrt(s)
            else:
                Li[j] = s / Lj[j]
    y = [0.0] * 6
    for i in range(6):
        s = d[i]
        Li = L[i]
        for k in range(i):
            s -= Li[k] * y[k]
        y[i] = s / Li[i]
    for i in range(5, -1, -1):
        s = y[i]
        for k in range(i + 1, 6):
            s -= L[k][i] * y[k]
        y[i] = s / L[i][i]
    return y


# ----------------------------------------------------------------------------- stage D: translation
def translation(asum, cnt, nb, qs, T_L, T_end):
    """World acceleration on knots -> per-axis Kalman filter + RTS smoother with motion prior."""
    bdt, kdt = P["bin_dt"], P["knot_dt"]
    per = max(1, int(round(kdt / bdt)))
    kdt = per * bdt
    K = int(math.ceil(T_end / kdt)) + 2
    KL = int(T_L / kdt)  # knots fully covered by data are < KL
    kacc = array("d", bytes(8 * 3 * K))
    kcnt = array("H", bytes(2 * K))
    gz = 0.0; ng = 0
    # world-frame specific force per bin (mid-bin attitude) -> knot means
    for b in range(nb):
        c = cnt[b]
        if not c:
            continue
        q0 = (qs[4 * b], qs[4 * b + 1], qs[4 * b + 2], qs[4 * b + 3])
        q1 = (qs[4 * b + 4], qs[4 * b + 5], qs[4 * b + 6], qs[4 * b + 7])
        qm = qnorm((q0[0] + q1[0], q0[1] + q1[1], q0[2] + q1[2], q0[3] + q1[3]))
        fw = qrot(qm, (asum[3 * b] / c, asum[3 * b + 1] / c, asum[3 * b + 2] / c))
        j = b // per
        if j >= K:
            continue
        kacc[3 * j] += fw[0]; kacc[3 * j + 1] += fw[1]; kacc[3 * j + 2] += fw[2]
        kcnt[j] += 1
        gz += fw[2]; ng += 1
    gz = gz / ng if ng else 9.81
    for j in range(K):
        if kcnt[j]:
            c = kcnt[j]
            kacc[3 * j] /= c; kacc[3 * j + 1] /= c; kacc[3 * j + 2] = kacc[3 * j + 2] / c - gz
    need = max(1, per // 3)
    adapt = P["adapt"]
    n_knot = max(1.0, kdt / INFO["dt_nom"])

    pos = array("d", bytes(8 * 3 * K))
    vel = array("d", bytes(8 * 3 * K))
    dt = kdt
    h = 0.5 * dt * dt
    # per-step model terms shared by the three axes: input known?, accel noise, bias random walk
    known = array("b", bytes(K))
    qa_ = array("d", bytes(8 * K))
    qbh_ = array("d", bytes(8 * K))
    qbz_ = array("d", bytes(8 * K))
    for j in range(K):
        tj = j * kdt
        kn = kcnt[j] >= need and j < KL + 1
        known[j] = 1 if kn else 0
        if kn:
            qa_[j] = P["sig_a"] ** 2 + (sigma_a(tj) ** 2 / n_knot if adapt else 0.0)
        else:
            qa_[j] = P["sig_gap"] ** 2
        sv = sev_at(tj) if adapt else 0.0
        qb = P["ba_rw"] + P["ba_k"] * sv
        qbh_[j] = qb * qb * dt
        qbz_[j] = (qb * qb + (P["ba_kz"] * sv) ** 2) * dt
    qc = P["sig_c"] ** 2 * dt
    g0, g1 = h, dt
    rz = P["zupt_sig"] ** 2
    zk = array("b", bytes(K))
    if STILL is not None:
        for j in range(K):
            k = int(j * kdt * 2.0)
            if 0 <= k < len(STILL) and STILL[k] and j * kdt <= T_L:
                zk[j] = 1
    store = array("d", bytes(8 * 14 * K))  # filtered mean (4) + covariance upper triangle (10)
    for ax in range(3):
        horiz = ax < 2
        sv_ = P["sig_v_h"] if horiz else P["sig_v_z"]
        sp_ = P["sig_p_h"] if horiz else P["sig_p_z"]
        # continuous-time prior: pseudo-measurement variance scales with 1/rate so the prior's information
        # per second is independent of the knot spacing (values are calibrated at 10 Hz knots)
        rv, rp = sv_ * sv_ * (0.1 / dt), sp_ * sp_ * (0.1 / dt)
        qb_ = qbh_ if horiz else qbz_
        # state x = [p, v, b, c]; covariance entries named cIJ (symmetric)
        x0 = x1 = x2 = x3 = 0.0
        c00, c01, c02, c03, c11, c12, c13, c22, c23, c33 = 1e-6, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.1, 0.0, rp
        for j in range(K):
            if j > 0:
                i = j - 1
                acc = kacc[3 * i + ax] if known[i] else 0.0
                qa = qa_[i]
                # mean: p += dt v - h b + h a ; v += -dt b + dt a
                x0 = x0 + dt * x1 - h * x2 + h * acc
                x1 = x1 - dt * x2 + dt * acc
                # covariance F C F' + Q with F = [[1, dt, -h, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                t00 = c00 + dt * c01 - h * c02; t01 = c01 + dt * c11 - h * c12; t02 = c02 + dt * c12 - h * c22
                t03 = c03 + dt * c13 - h * c23
                t11 = c11 - dt * c12; t12 = c12 - dt * c22; t13 = c13 - dt * c23
                t10 = c01 - dt * c02
                n00 = t00 + dt * t01 - h * t02
                n01 = t01 - dt * t02
                n02 = t02
                n03 = t03
                n11 = t11 - dt * t12
                n12 = t12
                n13 = t13
                c00 = n00 + g0 * g0 * qa; c01 = n01 + g0 * g1 * qa; c02 = n02; c03 = n03
                c11 = n11 + g1 * g1 * qa; c12 = n12; c13 = n13
                c22 = c22 + qb_[i]; c33 = c33 + qc
            # pseudo-measurement v = 0 (variance rv; much tighter at rest = zero-velocity update)
            S = c11 + (rz if zk[j] else rv)
            k0, k1, k2, k3 = c01 / S, c11 / S, c12 / S, c13 / S
            inn = -x1
            x0 += k0 * inn; x1 += k1 * inn; x2 += k2 * inn; x3 += k3 * inn
            p0, p1, p2, p3 = c01, c11, c12, c13
            c00 -= k0 * p0; c01 -= k0 * p1; c02 -= k0 * p2; c03 -= k0 * p3
            c11 -= k1 * p1; c12 -= k1 * p2; c13 -= k1 * p3
            c22 -= k2 * p2; c23 -= k2 * p3; c33 -= k3 * p3
            # pseudo-measurement p - c = 0 (variance rp)
            p0, p1, p2, p3 = c00 - c03, c01 - c13, c02 - c23, c03 - c33
            S = p0 - p3 + rp
            k0, k1, k2, k3 = p0 / S, p1 / S, p2 / S, p3 / S
            inn = -(x0 - x3)
            x0 += k0 * inn; x1 += k1 * inn; x2 += k2 * inn; x3 += k3 * inn
            c00 -= k0 * p0; c01 -= k0 * p1; c02 -= k0 * p2; c03 -= k0 * p3
            c11 -= k1 * p1; c12 -= k1 * p2; c13 -= k1 * p3
            c22 -= k2 * p2; c23 -= k2 * p3; c33 -= k3 * p3
            o = 14 * j
            store[o] = x0; store[o + 1] = x1; store[o + 2] = x2; store[o + 3] = x3
            store[o + 4] = c00; store[o + 5] = c01; store[o + 6] = c02; store[o + 7] = c03; store[o + 8] = c11
            store[o + 9] = c12; store[o + 10] = c13; store[o + 11] = c22; store[o + 12] = c23; store[o + 13] = c33
        # RTS smoother (mean only): xs_j = xf_j + Pf F' Pp^-1 (xs_{j+1} - xp_{j+1})
        o = 14 * (K - 1)
        s0, s1, s2, s3 = store[o], store[o + 1], store[o + 2], store[o + 3]
        pos[3 * (K - 1) + ax] = s0; vel[3 * (K - 1) + ax] = s1
        for j in range(K - 2, -1, -1):
            o = 14 * j
            f0, f1, f2, f3 = store[o], store[o + 1], store[o + 2], store[o + 3]
            c00, c01, c02, c03, c11, c12, c13, c22, c23, c33 = store[o + 4:o + 14]
            acc = kacc[3 * j + ax] if known[j] else 0.0
            qa = qa_[j]
            # prediction j -> j+1 (same algebra as the forward pass)
            xp0 = f0 + dt * f1 - h * f2 + h * acc
            xp1 = f1 - dt * f2 + dt * acc
            t00 = c00 + dt * c01 - h * c02; t01 = c01 + dt * c11 - h * c12; t02 = c02 + dt * c12 - h * c22
            t03 = c03 + dt * c13 - h * c23
            t11 = c11 - dt * c12; t12 = c12 - dt * c22; t13 = c13 - dt * c23
            a00 = t00 + dt * t01 - h * t02 + g0 * g0 * qa
            a01 = t01 - dt * t02 + g0 * g1 * qa
            a02 = t02; a03 = t03
            a11 = t11 - dt * t12 + g1 * g1 * qa
            a12 = t12; a13 = t13
            a22 = c22 + qb_[j]; a23 = c23; a33 = c33 + qc
            # y = Pp^-1 d  (Cholesky of the 4x4 predicted covariance)
            d0, d1, d2, d3 = s0 - xp0, s1 - xp1, s2 - f2, s3 - f3
            if a00 <= 0:
                s0, s1, s2, s3 = f0, f1, f2, f3
            else:
                l00 = math.sqrt(a00); l10 = a01 / l00; l20 = a02 / l00; l30 = a03 / l00
                e = a11 - l10 * l10
                if e <= 0:
                    s0, s1, s2, s3 = f0, f1, f2, f3
                else:
                    l11 = math.sqrt(e); l21 = (a12 - l20 * l10) / l11; l31 = (a13 - l30 * l10) / l11
                    e = a22 - l20 * l20 - l21 * l21
                    if e <= 0:
                        s0, s1, s2, s3 = f0, f1, f2, f3
                    else:
                        l22 = math.sqrt(e); l32 = (a23 - l30 * l20 - l31 * l21) / l22
                        e = a33 - l30 * l30 - l31 * l31 - l32 * l32
                        if e <= 0:
                            s0, s1, s2, s3 = f0, f1, f2, f3
                        else:
                            l33 = math.sqrt(e)
                            z0 = d0 / l00; z1 = (d1 - l10 * z0) / l11; z2 = (d2 - l20 * z0 - l21 * z1) / l22
                            z3 = (d3 - l30 * z0 - l31 * z1 - l32 * z2) / l33
                            y3 = z3 / l33; y2 = (z2 - l32 * y3) / l22; y1 = (z1 - l21 * y2 - l31 * y3) / l11
                            y0 = (z0 - l10 * y1 - l20 * y2 - l30 * y3) / l00
                            # u = F' y ; delta = Pf u
                            u0 = y0; u1 = dt * y0 + y1; u2 = -h * y0 - dt * y1 + y2; u3 = y3
                            s0 = f0 + c00 * u0 + c01 * u1 + c02 * u2 + c03 * u3
                            s1 = f1 + c01 * u0 + c11 * u1 + c12 * u2 + c13 * u3
                            s2 = f2 + c02 * u0 + c12 * u1 + c22 * u2 + c23 * u3
                            s3 = f3 + c03 * u0 + c13 * u1 + c23 * u2 + c33 * u3
            pos[3 * j + ax] = s0; vel[3 * j + ax] = s1
    return pos, vel, K, kdt, gz


# ----------------------------------------------------------------------------- pass E: output
def write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L):
    bdt = P["bin_dt"]
    isint = info["is_int"] and info["tscale"] == 1e-9
    t0i, t0f, tsc = info["first_ts"], info["first_tsf"], info["tscale"]
    # attitude after failure: hold the last smoothed attitude
    nbo = nb  # boundary index of the last bin
    n_out = 0

    def hermite(t):
        uk = t / kdt
        j = int(uk) if uk > 0 else 0
        if j >= K - 1:
            j = K - 2
        sk = min(1.0, max(0.0, uk - j))
        s2, s3 = sk * sk, sk * sk * sk
        h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, (s3 - 2 * s2 + sk) * kdt, -2 * s3 + 3 * s2, (s3 - s2) * kdt
        d00, d10, d01, d11 = (6 * s2 - 6 * sk) / kdt, 3 * s2 - 4 * sk + 1, (-6 * s2 + 6 * sk) / kdt, 3 * s2 - 2 * sk
        j3, j4 = 3 * j, 3 * j + 3
        p_ = [h00 * pos[j3 + c] + h10 * vel[j3 + c] + h01 * pos[j4 + c] + h11 * vel[j4 + c] for c in range(3)]
        v_ = [d00 * pos[j3 + c] + d10 * vel[j3 + c] + d01 * pos[j4 + c] + d11 * vel[j4 + c] for c in range(3)]
        return p_, v_
    # blind window (after the last valid sample): blind_tau < 0 -> the smoother's own prior-conditional
    # forecast; 0 -> hold the position at failure; > 0 -> velocity at failure decaying with time constant tau
    tau = P["blind_tau"]
    pL, vL = hermite(T_L)
    buf = []
    with open(out_path, "w", newline="\n") as fo:
        fo.write("# timestamp tx ty tz qx qy qz qw\n")
        with opener() as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                c = s[0]
                if not (c.isdigit() or c in "+-."):
                    continue
                tok = s.split(",", 1)[0].strip()
                try:
                    ti, tf = parse_ts(tok)
                except ValueError:
                    continue
                if ti is not None and t0i is not None:
                    t = (ti - t0i) * tsc
                else:
                    t = ((ti if ti is not None else tf) - (t0i if t0i is not None else t0f)) * tsc
                # timestamp string in seconds, exact
                if isint and ti is not None and ti >= 0:
                    ds = str(ti)
                    if len(ds) <= 9:
                        ds = "0" * (10 - len(ds)) + ds
                    ts_s = ds[:-9] + "." + ds[-9:]
                elif ti is not None:
                    ts_s = repr(ti * tsc)
                else:
                    ts_s = tok if tsc == 1.0 else repr(tf * tsc)
                # attitude: slerp-free normalized lerp between bin boundaries (bins are 20 ms)
                u = t / bdt
                if u >= nbo:
                    i = nbo; sfr = 0.0
                else:
                    i = int(u) if u > 0 else 0
                    sfr = u - i if u > 0 else 0.0
                if i >= nbo:
                    qw, qx, qy, qz = qs[4 * nbo], qs[4 * nbo + 1], qs[4 * nbo + 2], qs[4 * nbo + 3]
                else:
                    a0, a1, a2, a3 = qs[4 * i], qs[4 * i + 1], qs[4 * i + 2], qs[4 * i + 3]
                    b0, b1, b2, b3 = qs[4 * i + 4], qs[4 * i + 5], qs[4 * i + 6], qs[4 * i + 7]
                    if a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3 < 0:
                        b0, b1, b2, b3 = -b0, -b1, -b2, -b3
                    r = 1.0 - sfr
                    qw, qx, qy, qz = r * a0 + sfr * b0, r * a1 + sfr * b1, r * a2 + sfr * b2, r * a3 + sfr * b3
                n = 1.0 / math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
                qw *= n; qx *= n; qy *= n; qz *= n
                if qw < 0:
                    qw, qx, qy, qz = -qw, -qx, -qy, -qz
                if tau >= 0 and t > T_L:
                    f_ = tau * (1.0 - math.exp(-(t - T_L) / tau)) if tau > 0 else 0.0
                    buf.append("%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (ts_s, pL[0] + f_ * vL[0], pL[1] + f_ * vL[1],
                                                                          pL[2] + f_ * vL[2], qx, qy, qz, qw))
                    n_out += 1
                    continue
                # position: cubic Hermite between knots
                uk = t / kdt
                j = int(uk) if uk > 0 else 0
                if j >= K - 1:
                    j = K - 2
                sk = uk - j
                if sk < 0:
                    sk = 0.0
                elif sk > 1:
                    sk = 1.0
                s2, s3 = sk * sk, sk * sk * sk
                h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, (s3 - 2 * s2 + sk) * kdt, -2 * s3 + 3 * s2, (s3 - s2) * kdt
                j3, j4 = 3 * j, 3 * j + 3
                px = h00 * pos[j3] + h10 * vel[j3] + h01 * pos[j4] + h11 * vel[j4]
                py = h00 * pos[j3 + 1] + h10 * vel[j3 + 1] + h01 * pos[j4 + 1] + h11 * vel[j4 + 1]
                pz = h00 * pos[j3 + 2] + h10 * vel[j3 + 2] + h01 * pos[j4 + 2] + h11 * vel[j4 + 2]
                buf.append("%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (ts_s, px, py, pz, qx, qy, qz, qw))
                n_out += 1
                if len(buf) >= 2048:
                    fo.write("".join(buf)); buf = []
        if buf:
            fo.write("".join(buf))
    return n_out


# ----------------------------------------------------------------------------- main
INFO = {}
INIT = None
JAM = dict(L=0.0, gamma=1.7, T=1.0, sg=None, sa=None)
STILL = None  # per 0.5 s window: 1 if the body is at rest


def sev_at(t):
    """Estimated jamming severity at time t (0 = clean, ~1 = maximum of the official script)."""
    r = t / JAM["T"]
    if r <= 0:
        return 0.0
    return JAM["L"] * (r if r < 1 else 1.0) ** JAM["gamma"]


def sigma_g(t):
    a = JAM["sg"]
    return a[min(len(a) - 1, int(t))] if t > 0 else a[0]


def sigma_a(t):
    a = JAM["sa"]
    return a[min(len(a) - 1, int(t))] if t > 0 else a[0]


def run(inp, out_path, hook=None):
    """hook (development only): optional callable(stage, data) -> data, used by offline oracles."""
    global INFO, INIT, STILL
    opener, src = resolve_input(inp)
    info = survey(opener)
    INFO = info
    sg, sa, present, T_L, n_valid = noise_meter(opener, info)
    if n_valid < 10:
        raise SystemExit("fewer than 10 valid IMU rows")
    T_end = max(info["t_end"], T_L)
    L, gam = fit_severity(present, info["dt_nom"], T_L, T_end)
    JAM.update(L=L, gamma=gam, T=max(T_end, 1e-9), sg=smooth_levels(sg, T_L), sa=smooth_levels(sa, T_L))
    gsum, asum, cnt, nb, st, ws, nwin = preintegrate(opener, info, JAM["sg"], JAM["sa"], T_L)
    INIT = initial_attitude(gsum, asum, cnt, nb, sg)
    gmag = INIT[2]
    STILL = still_windows(ws, nwin, gmag) if P["still"] else None
    del ws
    qs, bg_end = attitude(gsum, asum, cnt, nb, T_L, gmag)
    if hook:
        qs = hook("attitude", dict(qs=qs, nb=nb, info=info))
    pos, vel, K, kdt, gz = translation(asum, cnt, nb, qs, T_L, T_end)
    n_out = write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L)
    log(f"src={src} rows={info['rows']} out={n_out} dt={info['dt_nom']:.5f} gyro_first={info['gyro_first']} "
        f"ascale={info['ascale']:.3f} T_L={T_L:.2f}s bins={nb} knots={K}")
    log(f"integrity {st}  still windows: {sum(STILL) if STILL is not None else 0} x 0.5 s")
    log(f"jamming L={L:.3f} gamma={gam:.3f} | white noise gyro {JAM['sg'][2]:.4f}->{JAM['sg'][max(0, int(T_L) - 1)]:.4f} rad/s, "
        f"accel {JAM['sa'][2]:.4f}->{JAM['sa'][max(0, int(T_L) - 1)]:.4f} m/s^2")
    log(f"init g={gmag:.4f} calm@{INIT[3]*P['bin_dt']:.2f}s var={INIT[4]:.2e} bg0={INIT[1]} bg_end={bg_end} gz={gz:.4f}")
    return n_out


def main(argv):
    pos_args = []
    for a in argv[1:]:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            if k not in P:
                raise SystemExit(f"unknown parameter {k}")
            P[k] = type(P[k])(float(v)) if not isinstance(P[k], str) else v
        else:
            pos_args.append(a)
    inp = pos_args[0] if pos_args else None
    out = pos_args[1] if len(pos_args) > 1 else "estimated_trajectory.txt"
    run(inp, out)


if __name__ == "__main__":
    main(sys.argv)
