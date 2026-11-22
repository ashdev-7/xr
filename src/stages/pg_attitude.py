# PathGuard stage file: initial attitude, attitude filter + RTS smoother.
# Embedded verbatim into the standalone src/main.py by tools/build_main.py and executed there, in main.py's namespace:
# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.
# Edit here, then run: python tools/build_main.py   (tests/test_build.py checks main.py is up to date).

def initial_attitude(gsum, asum, cnt, nb):
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
    # rotation taking body "up" u to world z: axis u x z = (u_y, -u_x, 0), angle atan2(|u x z|, u.z)
    ax, ay = u[1], -u[0]
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
    """Forward ESKF + error-state RTS smoother over bins; returns the smoothed quaternion at every bin
    boundary (nb + 1 entries) and the gyro bias at the end of the forward pass."""
    bdt = P["bin_dt"]
    every = max(1, int(P["tilt_every"]))
    dt_nom = INFO["dt_nom"]
    adapt = P["adapt"]
    n_tilt = max(1.0, every * bdt / dt_nom)  # samples averaged per gravity update

    # gyro for empty bins: linear interpolation between neighbouring filled bins (filled in place)
    filled = array("b", bytes(nb))
    glen = array("f", bytes(4 * nb))  # length [s] of the data gap each empty bin belongs to
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
            glen[j] = (b - a0) * bdt
            s = (j - left) / span if right != left else 0.0
            for k in range(3):
                gsum[3 * j + k] = (1 - s) * wl[k] + s * wr[k]
            # cnt stays 0: marks interpolated

    def gyro_var(b):
        """Rotation noise variance over one bin: measured white gyro noise integrated over the bin."""
        sg_ = max(sigma_g(b * bdt), P["gyro_floor"]) if adapt else P["gyro_floor"]
        v = sg_ * sg_ * dt_nom * bdt
        if not cnt[b]:
            # inside a data gap the rate is interpolated; the unknown angular acceleration makes the rotation
            # error grow like alpha * L per bin (total variance ~ alpha^2 L^3): ~2 deg for a 0.25 s burst,
            # ~0.8 rad for a 2 s signal loss, so gravity can re-level the attitude right after a long gap
            v = max(v * P["gap_gyro_k"] ** 2, (P["gap_alpha"] * glen[b] * bdt) ** 2)
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
