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
    bg_k=0.045,         # extra gyro bias RW per unit of jamming-induced gyro noise [1/sqrt(s)]
    gap_gyro_k=10.0,    # gyro noise inflation inside gaps
    bg0_sig=0.05,       # initial gyro bias uncertainty [rad/s] (EuRoC raw bias reaches ~0.08)
    # translation (per-second parameterisation, independent of knot_dt)
    sig_a=0.4,          # accel noise per knot [m/s^2] (scaled by sqrt(noise ratio))
    sig_gap=3.0,        # accel uncertainty where there is no data [m/s^2]
    ba_rw=5e-4,         # base accel bias random walk [m/s^2/sqrt(s)]
    ba_k=0.045,         # extra accel bias RW per unit of jamming-induced accel noise
    ba_kz=0.0,          # extra vertical bias RW per unit of jamming level (scale-factor drift)
    sig_c=0.2,          # workspace-centre random walk [m/sqrt(s)]
    sig_v_h=0.3,        # speed prior, horizontal [m/s]
    sig_v_z=0.1,        # speed prior, vertical [m/s]
    sig_p_h=1.0,        # workspace prior, horizontal [m]
    sig_p_z=0.5,        # workspace prior, vertical [m]
    # integrity
    spike_k=8.0,        # spike threshold in robust sigmas (must hold on all 6 channels)
    # features
    backward=1, adapt=1, repair=1, verbose=0,
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
    """Per-second robust white-noise level of gyro and accel (MAD of second differences),
    and the time of the last valid row (IMU failure)."""
    dt_nom = info["dt_nom"]
    win = 1.0
    sg, sa = array("f"), array("f")
    bg, ba = [], []
    w_idx = 0
    p1 = p2 = None  # previous two valid rows (t, 6 values)
    t_last_valid = 0.0
    last = None
    n_valid = 0
    for r in row_reader(opener, info):
        t = r[0]
        while t >= (w_idx + 1) * win:
            sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba)); bg, ba = [], []; w_idx += 1
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
    sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba))
    return sg, sa, t_last_valid, n_valid


def _mad_sigma(x):
    if len(x) < 30:
        return -1.0
    x.sort()
    return 1.4826 * x[len(x) // 2] / math.sqrt(6.0)


def fit_ramp(sig, T):
    """Fit sigma(t)^2 = s0^2 + A^2 (t/T)^(2 gamma) to per-second noise levels (weighted LS, grid on gamma).
    Returns (s0, A, gamma). This is the jamming level: the official degradation scales every
    impairment (white noise, bias drift, scale drift, spikes, drops) by one severity ramp."""
    pts = [((i + 0.5) / max(T, 1e-9), s * s) for i, s in enumerate(sig) if s > 0 and (i + 0.5) <= T]
    if len(pts) < 5:
        return (max([s for s in sig if s > 0] or [0.0]), 0.0, 1.7)
    best = None
    for gi in range(4, 25):
        g = gi * 0.125
        # linear LS for y = c0 + c1 * x,  x = r^(2g); weights 1/y (relative errors)
        S = Sx = Sy = Sxx = Sxy = 0.0
        for r, y in pts:
            x = min(r, 1.0) ** (2 * g)
            w = 1.0 / (y * y + 1e-30)
            S += w; Sx += w * x; Sy += w * y; Sxx += w * x * x; Sxy += w * x * y
        den = S * Sxx - Sx * Sx
        if abs(den) < 1e-300:
            continue
        c1 = (S * Sxy - Sx * Sy) / den
        c0 = (Sy - c1 * Sx) / S
        c1 = max(c1, 0.0)
        c0 = max(c0, 1e-12)
        e = 0.0
        for r, y in pts:
            m = c0 + c1 * min(r, 1.0) ** (2 * g)
            e += (math.log(y) - math.log(m)) ** 2
        if best is None or e < best[0]:
            best = (e, math.sqrt(c0), math.sqrt(c1), g)
    if best is None:
        return (0.0, 0.0, 1.7)
    return best[1], best[2], best[3]


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

    def emit(r):
        t = r[0]
        if t > T_L + 1e-9:
            return
        b = int(t / bdt)
        if b >= nb:
            b = nb - 1
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
    return gsum, asum, cnt, nb, stats


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
        qb2: bias RW var added."""
        bg = self.bg
        q = self.q
        self.q = qnorm(qmul(q, qexp((w[0] - bg[0]) * dt, (w[1] - bg[1]) * dt, (w[2] - bg[2]) * dt)))
        # dtheta' = dtheta - dt * R dbg  (world-frame error, F = [[I, -dt R], [0, I]])
        R = qmat(q)
        A, B, C = self.A, self.B, self.C
        RC = m3mul(R, C)
        RB = m3mul(R, m3T(B))          # R B^T  (3x3)
        # A' = A - dt (R B^T + B R^T) + dt^2 R C R^T + qg2 I
        RCRt = m3mul(RC, m3T(R))
        An = tuple(tuple(A[i][j] - dt * (RB[i][j] + RB[j][i]) + dt * dt * RCRt[i][j] + (qg2 if i == j else 0.0)
                         for j in range(3)) for i in range(3))
        Bn = m3add(B, RC, -dt)         # B' = B - dt R C
        Cn = ((C[0][0] + qb2, C[0][1], C[0][2]), (C[1][0], C[1][1] + qb2, C[1][2]), (C[2][0], C[2][1], C[2][2] + qb2))
        self.A, self.B, self.C = An, Bn, Cn

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
    lim = min(nb, int(15.0 / bdt))
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


def attitude(gsum, asum, cnt, nb, sg, sa, ramp_g, ramp_a, T_L, gmag):
    """Forward + backward ESKF over bins; returns smoothed quaternion at every bin boundary (nb+1)."""
    bdt = P["bin_dt"]
    every = max(1, int(P["tilt_every"]))
    dt_nom = INFO["dt_nom"]
    nw = len(sg)
    s0g, Ag, gg = ramp_g
    adapt = P["adapt"]

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
        """Rotation noise variance over one bin."""
        w = min(nw - 1, int(b * bdt))
        if adapt and Ag > 0:
            r = min(1.0, b * bdt / max(T_L, 1e-9))
            s = math.sqrt(s0g * s0g + (Ag * r ** gg) ** 2)
        else:
            s = sg[w] if sg[w] > 0 else s0g
        s = max(s, P["gyro_floor"])
        c = cnt[b]
        v = s * s * dt_nom * bdt  # white rate noise integrated over the bin
        if not c:
            v *= P["gap_gyro_k"] ** 2
        return v

    def bias_var(b):
        r = min(1.0, b * bdt / max(T_L, 1e-9))
        q = P["bg_rw"] + (P["bg_k"] * Ag * r ** gg if adapt else 0.0)
        return q * q * bdt

    def tilt_sig(f, b):
        n = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
        dev = abs(n - gmag)
        if dev > P["tilt_gate"]:
            return None
        s2 = P["tilt_sig"] ** 2 + (P["tilt_k"] * dev) ** 2
        if adapt and ramp_a[1] > 0:
            r = min(1.0, b * bdt / max(T_L, 1e-9))
            s0a, Aa, ga = ramp_a
            ratio = (s0a * s0a + (Aa * r ** ga) ** 2) / max(s0a * s0a, 1e-12)
            s2 *= math.sqrt(ratio)
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
def translation(asum, cnt, nb, qs, T_L, T_end, ramp_a):
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
    s0a, Aa, ga = ramp_a
    adapt = P["adapt"]

    def noise_ratio(j):
        if not adapt or Aa <= 0:
            return 1.0
        r = min(1.0, j * kdt / max(T_L, 1e-9))
        return math.sqrt((s0a * s0a + (Aa * r ** ga) ** 2) / max(s0a * s0a, 1e-12))

    def sev(j):
        r = min(1.0, j * kdt / max(T_L, 1e-9))
        return r ** ga

    pos = array("d", bytes(8 * 3 * K))
    vel = array("d", bytes(8 * 3 * K))
    xs_store = array("d", bytes(8 * 4 * K))
    Ps_store = array("d", bytes(8 * 10 * K))
    dt = kdt
    F = ((1.0, dt, -0.5 * dt * dt, 0.0), (0.0, 1.0, -dt, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for ax in range(3):
        horiz = ax < 2
        sv = (P["sig_v_h"] if horiz else P["sig_v_z"])
        sp = (P["sig_p_h"] if horiz else P["sig_p_z"])
        rv, rp = sv * sv, sp * sp

        def model(j):
            """Input accel and process noise for the step j -> j+1."""
            known = kcnt[j] >= need and j < KL + 1
            a = kacc[3 * j + ax] if known else 0.0
            if known:
                sa = P["sig_a"] * math.sqrt(noise_ratio(j))
            else:
                sa = P["sig_gap"]
            qb = P["ba_rw"] + (P["ba_k"] * Aa * sev(j) if adapt else 0.0)
            if not horiz and adapt:
                qb = math.sqrt(qb * qb + (P["ba_kz"] * sev(j)) ** 2)
            return a, sa * sa, qb * qb * dt, P["sig_c"] ** 2 * dt

        x = [0.0, 0.0, 0.0, 0.0]
        C = [[1e-6, 0, 0, 0], [0, 0.5, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, rp]]
        for j in range(K):
            if j > 0:
                a, qa, qb, qc = model(j - 1)
                x, C = kf_predict(x, C, F, a, dt, qa, qb, qc)
            # motion prior pseudo-measurements: v = 0, p - c = 0
            x, C = kf_update(x, C, (0.0, 1.0, 0.0, 0.0), 0.0, rv)
            x, C = kf_update(x, C, (1.0, 0.0, 0.0, -1.0), 0.0, rp)
            xs_store[4 * j:4 * j + 4] = array("d", x)
            Ps_store[10 * j:10 * j + 10] = array("d", (C[0][0], C[0][1], C[0][2], C[0][3], C[1][1], C[1][2], C[1][3], C[2][2], C[2][3], C[3][3]))
        # RTS (mean only)
        xs = list(xs_store[4 * (K - 1):4 * K])
        pos[3 * (K - 1) + ax] = xs[0]; vel[3 * (K - 1) + ax] = xs[1]
        for j in range(K - 2, -1, -1):
            xf = list(xs_store[4 * j:4 * j + 4])
            p = Ps_store[10 * j:10 * j + 10]
            Pf = [[p[0], p[1], p[2], p[3]], [p[1], p[4], p[5], p[6]], [p[2], p[5], p[7], p[8]], [p[3], p[6], p[8], p[9]]]
            a, qa, qb, qc = model(j)
            xp, Pp = kf_predict(xf, Pf, F, a, dt, qa, qb, qc)
            Pi = inv4(Pp)
            if Pi is None:
                xs = xf
            else:
                # G = Pf F^T Pp^-1
                PFt = [[sum(Pf[r][k] * F[c][k] for k in range(4)) for c in range(4)] for r in range(4)]
                G = [[sum(PFt[r][k] * Pi[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
                d = [xs[k] - xp[k] for k in range(4)]
                xs = [xf[r] + sum(G[r][k] * d[k] for k in range(4)) for r in range(4)]
            pos[3 * j + ax] = xs[0]; vel[3 * j + ax] = xs[1]
    return pos, vel, K, kdt, gz


def kf_predict(x, C, F, a, dt, qa, qb, qc):
    nx = [sum(F[i][k] * x[k] for k in range(4)) for i in range(4)]
    nx[0] += 0.5 * dt * dt * a
    nx[1] += dt * a
    T = [[sum(F[i][k] * C[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    N = [[sum(T[i][k] * F[j][k] for k in range(4)) for j in range(4)] for i in range(4)]
    g0, g1 = 0.5 * dt * dt, dt
    N[0][0] += g0 * g0 * qa; N[0][1] += g0 * g1 * qa; N[1][0] += g0 * g1 * qa; N[1][1] += g1 * g1 * qa
    N[2][2] += qb; N[3][3] += qc
    return nx, N


def kf_update(x, C, h, z, r):
    Ph = [sum(C[i][k] * h[k] for k in range(4)) for i in range(4)]
    S = r + sum(h[i] * Ph[i] for i in range(4))
    inn = z - sum(h[i] * x[i] for i in range(4))
    k = [p / S for p in Ph]
    x = [x[i] + k[i] * inn for i in range(4)]
    C = [[C[i][j] - k[i] * Ph[j] for j in range(4)] for i in range(4)]
    return x, C


def inv4(A):
    M = [list(A[i]) + [1.0 if j == i else 0.0 for j in range(4)] for i in range(4)]
    for c in range(4):
        p = max(range(c, 4), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-300:
            return None
        if p != c:
            M[p], M[c] = M[c], M[p]
        d = M[c][c]
        M[c] = [v / d for v in M[c]]
        for r in range(4):
            if r != c:
                f = M[r][c]
                if f:
                    M[r] = [M[r][k] - f * M[c][k] for k in range(8)]
    return [row[4:] for row in M]


# ----------------------------------------------------------------------------- pass E: output
def write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L):
    bdt = P["bin_dt"]
    isint = info["is_int"] and info["tscale"] == 1e-9
    t0i, t0f, tsc = info["first_ts"], info["first_tsf"], info["tscale"]
    # attitude after failure: hold the last smoothed attitude
    nbo = nb  # boundary index of the last bin
    n_out = 0
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


def run(inp, out_path):
    global INFO, INIT
    opener, src = resolve_input(inp)
    info = survey(opener)
    INFO = info
    sg, sa, T_L, n_valid = noise_meter(opener, info)
    if n_valid < 10:
        raise SystemExit("fewer than 10 valid IMU rows")
    ramp_g = fit_ramp(sg, T_L)
    ramp_a = fit_ramp(sa, T_L)
    gsum, asum, cnt, nb, st = preintegrate(opener, info, sg, sa, T_L)
    INIT = initial_attitude(gsum, asum, cnt, nb, sg)
    gmag = INIT[2]
    qs, bg_end = attitude(gsum, asum, cnt, nb, sg, sa, ramp_g, ramp_a, T_L, gmag)
    # last row time (end of the blind window) from survey: approximate by the row reader in output pass
    T_end = max(info["t_end"], T_L)
    pos, vel, K, kdt, gz = translation(asum, cnt, nb, qs, T_L, T_end, ramp_a)
    n_out = write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L)
    log(f"src={src} rows={info['rows']} out={n_out} dt={info['dt_nom']:.5f} gyro_first={info['gyro_first']} "
        f"ascale={info['ascale']:.3f} T_L={T_L:.2f}s bins={nb} knots={K}")
    log(f"integrity {st}")
    log(f"jamming gyro s0={ramp_g[0]:.4g} A={ramp_g[1]:.4g} gamma={ramp_g[2]:.3g} | accel s0={ramp_a[0]:.4g} "
        f"A={ramp_a[1]:.4g} gamma={ramp_a[2]:.3g}")
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
