# PathGuard stage file: pass 2 (one pose per input row) and the pipeline driver run().
# Embedded verbatim into the standalone src/main.py by tools/build_main.py and executed there, in main.py's namespace:
# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.
# Edit here, then run: python tools/build_main.py   (tests/test_build.py checks main.py is up to date).

def write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L, wL=(0.0, 0.0, 0.0)):
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
    atau = P["att_tau"]
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
                if "," not in s:  # tolerate ';' or tab/whitespace delimiters (EuRoC uses ',')
                    s = s.replace(";", ",") if ";" in s else ",".join(s.split())
                k = s.find(",")
                tok = (s[:k] if k >= 0 else s).strip()
                try:  # parse_ts, inlined (hot loop)
                    ti, tf = int(tok), None
                except ValueError:
                    try:
                        ti, tf = None, float(tok)
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
                    # blind window: continue the last body rate, decaying with time constant att_tau
                    qw, qx, qy, qz = qs[4 * nbo], qs[4 * nbo + 1], qs[4 * nbo + 2], qs[4 * nbo + 3]
                    if atau > 0 and t > T_L:
                        fa = atau * (1.0 - math.exp(-(t - T_L) / atau))
                        qw, qx, qy, qz = qmul((qw, qx, qy, qz), qexp(wL[0] * fa, wL[1] * fa, wL[2] * fa))
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
                    fo.write(_no_negative_zero("".join(buf))); buf = []
        if buf:
            fo.write(_no_negative_zero("".join(buf)))
    return n_out


def _no_negative_zero(text):
    """Print tiny negative values as 0.000000 rather than -0.000000 (fixed-width tokens, so exact)."""
    for _ in range(2):  # twice: adjacent tokens share a space, and str.replace does not overlap matches
        text = text.replace(" -0.000000000", " 0.000000000").replace(" -0.000000 ", " 0.000000 ")
    return text


# ----------------------------------------------------------------------------- main
INFO = {}
INIT = None
JAM = dict(L=0.0, gamma=1.7, T=1.0, sg=None, sa=None)
STILL = None  # per 0.5 s window: 1 if the body is at rest
PLATFORM = 0  # 1 rotorcraft, 0 handheld/ground
DIAG = {}


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
    global INFO, INIT, STILL, PLATFORM
    opener, src = resolve_input(inp)
    info = survey(opener)
    INFO = info
    d = ingest(opener, info)
    T_L, T_end, st = d["T_L"], max(d["t_end"], d["T_L"]), d["stats"]
    if st["valid"] < 10:
        raise ValueError("fewer than 10 valid IMU rows")
    sg = d["sg"]
    # platform class from the rotor-vibration signature (first half of the flight, before heavy jamming)
    half = [x for i, x in enumerate(d["sa"]) if x > 0 and i < 0.5 * T_L]
    vib = sorted(half)[len(half) // 2] if half else 0.0
    plat = int(P["platform"]) if P["platform"] >= 0 else (1 if vib > P["vib_thr"] else 0)
    PLATFORM = plat
    for k, v in PRESETS[plat].items():
        if k not in CLI_SET:
            P[k] = v
    L, gam = fit_severity(d["present"], info["dt_nom"], T_L, T_end)
    JAM.update(L=L, gamma=gam, T=max(T_end, 1e-9), sg=smooth_levels(sg, T_L), sa=smooth_levels(d["sa"], T_L))
    gsum, asum, cnt, nb, ws, nwin = d["gsum"], d["asum"], d["cnt"], d["nb"], d["ws"], d["nwin"]
    del d
    INIT = initial_attitude(gsum, asum, cnt, nb)
    gmag = INIT[2]
    STILL = still_windows(ws, nwin, gmag) if P["still"] else None
    del ws
    qs, bg_end = attitude(gsum, asum, cnt, nb, T_L, gmag)
    if hook:
        qs = hook("attitude", dict(qs=qs, nb=nb, info=info))
    pos, vel, K, kdt, gz = translation(asum, cnt, nb, qs, T_L, T_end)
    # body rate at the failure: mean bias-corrected gyro over the last 0.5 s of data (for the blind attitude)
    wL, nw_ = [0.0, 0.0, 0.0], 0
    for b in range(nb - 1, max(-1, nb - 1 - int(0.5 / P["bin_dt"])), -1):
        c = cnt[b]
        if c:
            for k in range(3):
                wL[k] += gsum[3 * b + k] / c - bg_end[k]
            nw_ += 1
    wL = [x / nw_ for x in wL] if nw_ else wL
    n_out = write_output(opener, info, out_path, qs, nb, pos, vel, K, kdt, T_L, wL)
    log(f"src={src} rows={st['rows']} out={n_out} dt={info['dt_nom']:.5f} gyro_first={info['gyro_first']} "
        f"ascale={info['ascale']:.3f} T_L={T_L:.2f}s bins={nb} knots={K}")
    log(f"platform {'rotorcraft' if plat else 'handheld'} (accel vibration {vib:.3f} m/s^2) {DIAG}")
    log(f"integrity {st}  still windows: {sum(STILL) if STILL is not None else 0} x 0.5 s")
    i0, i1 = min(2, len(JAM["sg"]) - 1), min(max(0, int(T_L) - 1), len(JAM["sg"]) - 1)
    log(f"jamming L={L:.3f} gamma={gam:.3f} | white noise gyro {JAM['sg'][i0]:.4f}->{JAM['sg'][i1]:.4f} rad/s, "
        f"accel {JAM['sa'][i0]:.4f}->{JAM['sa'][i1]:.4f} m/s^2")
    log(f"init g={gmag:.4f} calm@{INIT[3]*P['bin_dt']:.2f}s var={INIT[4]:.2e} bg0={INIT[1]} bg_end={bg_end} gz={gz:.4f}")
    return n_out
