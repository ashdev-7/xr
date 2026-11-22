# PathGuard stage file: translation: per-axis Kalman filter + RTS smoother on 10 Hz knots.
# Embedded verbatim into the standalone src/main.py by tools/build_main.py and executed there, in main.py's namespace:
# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.
# Edit here, then run: python tools/build_main.py   (tests/test_build.py checks main.py is up to date).

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
    kfb = array("d", bytes(8 * 3 * K))  # body-frame specific force (for the rotor-drag model)
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
        fb = (asum[3 * b] / c, asum[3 * b + 1] / c, asum[3 * b + 2] / c)
        fw = qrot(qm, fb)
        j = b // per
        if j >= K:
            continue
        kacc[3 * j] += fw[0]; kacc[3 * j + 1] += fw[1]; kacc[3 * j + 2] += fw[2]
        kfb[3 * j] += fb[0]; kfb[3 * j + 1] += fb[1]; kfb[3 * j + 2] += fb[2]
        kcnt[j] += 1
        gz += fw[2]; ng += 1
    gz = gz / ng if ng else 9.81
    for j in range(K):
        if kcnt[j]:
            c = kcnt[j]
            kacc[3 * j] /= c; kacc[3 * j + 1] /= c; kacc[3 * j + 2] = kacc[3 * j + 2] / c - gz
            kfb[3 * j] /= c; kfb[3 * j + 1] /= c; kfb[3 * j + 2] /= c
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
        else:  # no data: inside the log (dropout) or after the failure (blind window) use separate uncertainties
            qa_[j] = (P["sig_gap"] if tj > T_L else P["sig_gap_in"]) ** 2
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
    # ---- rotor-drag velocity measurement (rotorcraft, rotors spinning, not at rest)
    # A multirotor's thrust acts along one body axis u, so the specific force perpendicular to u is rotor
    # drag: f_perp = -mu v_perp (Leishman et al. 2014). u = mean specific-force direction while airborne
    # (this also absorbs a constant accelerometer bias). The body velocity is rotated to the world frame
    # with the smoothed attitude and observed on the horizontal axes.
    dvel = None
    if PLATFORM == 1 and P["drag"]:
        air = array("b", bytes(K))
        su = [0.0, 0.0, 0.0]
        for j in range(K):
            if kcnt[j] >= need and j * kdt <= T_L and not zk[j] and sigma_a(j * kdt) > P["drag_vib"]:
                air[j] = 1
                su[0] += kfb[3 * j]; su[1] += kfb[3 * j + 1]; su[2] += kfb[3 * j + 2]
        nu = math.sqrt(su[0] ** 2 + su[1] ** 2 + su[2] ** 2)
        if nu > 0 and sum(air) * kdt > 5.0:
            u = (su[0] / nu, su[1] / nu, su[2] / nu)
            dvel = array("d", bytes(8 * 2 * K))
            mu = P["drag_mu"]
            for j in range(K):
                if not air[j]:
                    continue
                f0, f1, f2 = kfb[3 * j], kfb[3 * j + 1], kfb[3 * j + 2]
                fu = f0 * u[0] + f1 * u[1] + f2 * u[2]
                vb = (-(f0 - fu * u[0]) / mu, -(f1 - fu * u[1]) / mu, -(f2 - fu * u[2]) / mu)
                bm = min(nb, j * per + per // 2)
                vw = qrot((qs[4 * bm], qs[4 * bm + 1], qs[4 * bm + 2], qs[4 * bm + 3]), vb)
                dvel[2 * j] = vw[0]; dvel[2 * j + 1] = vw[1]
            DIAG["drag_knots"] = sum(air); DIAG["thrust_axis"] = u
    rd = P["drag_sig"] ** 2 * (0.1 / kdt)
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
            # rotor-drag velocity measurement v = v_drag (horizontal axes, airborne rotorcraft)
            if dvel is not None and ax < 2 and air[j]:
                S = c11 + rd
                k0, k1, k2, k3 = c01 / S, c11 / S, c12 / S, c13 / S
                inn = dvel[2 * j + ax] - x1
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
