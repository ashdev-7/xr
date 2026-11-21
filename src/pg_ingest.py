# PathGuard stage file: pass 1: survey of the input and the single streaming pass (integrity, jamming meter, preintegration, rest statistics).
# Executed by main.py into main.py's own namespace (see _load_stages there), not imported as a module:
# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.
# Do not run directly.

def survey(opener, nmax=4000):
    """Pre-read up to nmax data rows: header, column order, units, nominal sample period."""
    header = None
    rows = 0
    first_ts = first_tsf = prev = None
    is_int = True
    dts, n13, n46 = [], [], []
    with opener() as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if not (s[0].isdigit() or s[0] in "+-."):
                if header is None and rows == 0:
                    header = s
                continue
            if " " in s or "\t" in s:
                s = s.replace(" ", "").replace("\t", "")
            parts = s.split(",")
            try:
                ti, tf = parse_ts(parts[0])
            except ValueError:
                continue
            rows += 1
            if ti is None:
                is_int = False
            if rows == 1:
                first_ts, first_tsf = ti, tf
            cur = ti if ti is not None else tf
            if prev is not None and cur > prev:
                dts.append(cur - prev)
            prev = cur
            if len(parts) >= 7:
                try:
                    v = [float(x) for x in parts[1:7]]
                except ValueError:
                    v = None
                if v and all(math.isfinite(x) for x in v) and any(v):
                    n13.append(math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
                    n46.append(math.sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]))
            if rows >= nmax:
                break
    if rows < 3:
        raise ValueError("input has fewer than 3 IMU rows")
    # timestamp unit -> seconds (same rule as the official degradation script)
    dts.sort()
    md = dts[len(dts) // 2] if dts else 1
    tscale = 1.0 if md < 1.0 else 1e-3 if md < 1e3 else 1e-6 if md < 1e6 else 1e-9
    dt_nom = md * tscale if dts else 0.005
    # column layout: header names first, gravity magnitude as fallback
    gyro_first = True
    if header:
        h = [x.strip().lower() for x in header.lstrip("#").split(",")]
        kind = lambda n: ("g" if n.startswith("w") or "gyr" in n or "rad" in n else
                          "a" if n.startswith("a") or "acc" in n or "m s^-2" in n or "m/s" in n else "?")
        if len(h) < 7 or "?" in (kind(h[1]), kind(h[4])):
            header = None
        elif kind(h[1]) == "a" and kind(h[4]) == "g":
            gyro_first = False
    med = lambda x: sorted(x)[len(x) // 2] if x else 0.0
    if not header and n13:  # the accelerometer triple carries gravity (~9.8 m/s^2 or ~1 g)
        gs = lambda m: min(abs(m - 9.80665), abs(m - 1.0) * 9.80665)
        gyro_first = gs(med(n46)) <= gs(med(n13))
    ma = med(n46) if gyro_first else med(n13)
    return dict(header=header, tscale=tscale, dt_nom=dt_nom, gyro_first=gyro_first,
                ascale=9.80665 if 0.5 < ma < 2.0 else 1.0, is_int=is_int, first_ts=first_ts, first_tsf=first_tsf)


# ----------------------------------------------------------------------------- pass 1b: one-pass ingest
def ingest(opener, info):
    """Single streaming pass over all rows:
      * integrity: NaN/short rows, dead (all-zero) rows, frozen/duplicate rows (exact repeats), spikes
      * jamming meter: per-second robust white-noise level of gyro and accel (MAD of second
        differences) and unique timestamps present (-> packet loss)
      * preintegration of valid rows into bins (sum of gyro and accel, count)
      * 0.5 s window moments for rest detection
    Spike rule (signature of the jamming model): the SAME sample jumps on all six channels at once by
    more than spike_k robust sigmas from the median of its two neighbours on each side (measured on
    labelled official degradations: precision 0.97, recall 0.85; a per-channel rule: precision 0.16)."""
    t0i, t0f, tsc = info["first_ts"], info["first_tsf"], info["tscale"]
    gf, asc, dt_nom = info["gyro_first"], info["ascale"], info["dt_nom"]
    bdt = P["bin_dt"]
    kk, repair = P["spike_k"], P["repair"]
    isfin = math.isfinite
    gsum, asum, cnt = array("d"), array("d"), array("H")   # per bin
    ws = array("d")                                          # per 0.5 s window: n, sum w, sum w^2, sum f, sum f^2
    sg, sa, present = array("f"), array("f"), array("H")    # per second
    bg, ba = [], []
    w_idx = npres = 0
    last_t = last = p1 = p2 = None
    t_last_valid = t_acc = 0.0
    st = dict(rows=0, nan=0, dead=0, frozen=0, spikes=0, valid=0)
    buf = []
    sig_g, sig_a = 0.01, 0.1
    lo_dt, hi_dt = 0.5 * dt_nom, 1.5 * dt_nom

    def emit(r):
        tt = r[0]
        b = int(tt / bdt)
        while len(cnt) <= b:
            gsum.extend((0.0, 0.0, 0.0)); asum.extend((0.0, 0.0, 0.0)); cnt.append(0)
        i3 = 3 * b
        gsum[i3] += r[1]; gsum[i3 + 1] += r[2]; gsum[i3 + 2] += r[3]
        asum[i3] += r[4]; asum[i3 + 1] += r[5]; asum[i3 + 2] += r[6]
        if cnt[b] < 65535:
            cnt[b] += 1
        k = int(tt * 2.0)
        while len(ws) <= 13 * k:
            ws.extend((0.0,) * 13)
        o = 13 * k
        _, g0, g1, g2, a0, a1, a2 = r
        ws[o] += 1.0
        ws[o + 1] += g0; ws[o + 2] += g1; ws[o + 3] += g2
        ws[o + 4] += g0 * g0; ws[o + 5] += g1 * g1; ws[o + 6] += g2 * g2
        ws[o + 7] += a0; ws[o + 8] += a1; ws[o + 9] += a2
        ws[o + 10] += a0 * a0; ws[o + 11] += a1 * a1; ws[o + 12] += a2 * a2
        st["valid"] += 1

    def spike(i):
        r = buf[i]
        o = [buf[j] for j in range(5) if j != i]
        for c in range(1, 7):
            v = sorted((o[0][c], o[1][c], o[2][c], o[3][c]))
            if abs(r[c] - 0.5 * (v[1] + v[2])) <= kk * (sig_g if c <= 3 else sig_a):
                return False
        return True

    with opener() as f:
        for line in f:
            s = line.strip()
            if not s or not (s[0].isdigit() or s[0] in "+-."):
                continue
            if " " in s or "\t" in s:
                s = s.replace(" ", "").replace("\t", "")
            parts = s.split(",")
            try:
                ti, tf = parse_ts(parts[0])
            except ValueError:
                continue
            st["rows"] += 1
            t = (ti - t0i) * tsc if (ti is not None and t0i is not None) else \
                ((ti if ti is not None else tf) - (t0i if t0i is not None else t0f)) * tsc
            if t < 0.0 or t > t_last_valid + 60.0 or (last_t is not None and t < last_t):
                st["nan"] += 1  # out-of-order or glitched timestamp: still gets a pose, not used for estimation
                continue
            t_acc = t
            while t >= w_idx + 1.0:  # close the per-second window
                sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba)); present.append(min(npres, 65535))
                if sg[-1] > 0:
                    sig_g = sg[-1]
                if sa[-1] > 0:
                    sig_a = sa[-1]
                bg, ba = [], []; npres = 0; w_idx += 1
            if t != last_t:
                npres += 1
                last_t = t
            try:
                v = (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6]))
            except (ValueError, IndexError):
                st["nan"] += 1
                continue
            if not (isfin(v[0]) and isfin(v[1]) and isfin(v[2]) and isfin(v[3]) and isfin(v[4]) and isfin(v[5])):
                st["nan"] += 1
                continue
            if not (v[0] or v[1] or v[2] or v[3] or v[4] or v[5]):
                st["dead"] += 1
                last = v
                continue
            if v == last:
                st["frozen"] += 1
                continue
            last = v
            r = (t, v[0], v[1], v[2], v[3] * asc, v[4] * asc, v[5] * asc) if gf else \
                (t, v[3], v[4], v[5], v[0] * asc, v[1] * asc, v[2] * asc)
            t_last_valid = t
            if p1 is not None and p2 is not None:  # second differences for the noise meter
                d1, d2 = t - p1[0], p1[0] - p2[0]
                if lo_dt < d1 < hi_dt and lo_dt < d2 < hi_dt:
                    bg.extend((abs(r[1] - 2 * p1[1] + p2[1]), abs(r[2] - 2 * p1[2] + p2[2]), abs(r[3] - 2 * p1[3] + p2[3])))
                    ba.extend((abs(r[4] - 2 * p1[4] + p2[4]), abs(r[5] - 2 * p1[5] + p2[5]), abs(r[6] - 2 * p1[6] + p2[6])))
            p2, p1 = p1, r
            if not repair:
                emit(r)
                continue
            buf.append(r)
            if len(buf) == 5:
                if spike(2):
                    st["spikes"] += 1
                    del buf[2]
                else:
                    emit(buf.pop(0))
    for r in buf:  # the last rows before the failure have no two-sided window
        emit(r)
    sg.append(_mad_sigma(bg)); sa.append(_mad_sigma(ba)); present.append(min(npres, 65535))
    nb = int(t_last_valid / bdt) + 1
    while len(cnt) < nb:
        gsum.extend((0.0, 0.0, 0.0)); asum.extend((0.0, 0.0, 0.0)); cnt.append(0)
    return dict(sg=sg, sa=sa, present=present, T_L=t_last_valid, t_end=t_acc, gsum=gsum, asum=asum, cnt=cnt,
                nb=nb, ws=ws, nwin=len(ws) // 13, stats=st)


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

    # expected missing fraction miss(s) = 0.06 s + 0.0015 s (5 + 22.5 s) = 0.0675 s + 0.03375 s^2
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
