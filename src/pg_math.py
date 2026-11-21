# PathGuard stage file: quaternion / 3x3 helpers and the attitude error-state Kalman filter (class Att).
# Executed by main.py into main.py's own namespace (see _load_stages there), not imported as a module:
# it uses the imports (math, array, os, sys), parameters P and helpers defined in main.py.
# Do not run directly.

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
