"""Unit test: the attitude ESKF must recover a known gyro bias and attitude on a tumbling body
with gravity-only specific force (no translation acceleration)."""
import math, os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import main as M


def simulate(T=120.0, dt=0.02, bias=(0.01, -0.02, 0.03), seed=0, tilt_amp=0.6):
    rnd = random.Random(seed)
    q = M.qexp(0.3, -0.2, 0.0)
    out = []
    for k in range(int(T / dt)):
        t = k * dt
        w = (tilt_amp * math.sin(0.7 * t), tilt_amp * math.cos(0.5 * t), 0.8 * math.sin(0.3 * t))  # body rate
        f = M.qrot((q[0], -q[1], -q[2], -q[3]), (0.0, 0.0, 9.81))                           # R^T g
        wm = tuple(w[i] + bias[i] + rnd.gauss(0, 0.002) for i in range(3))
        fm = tuple(f[i] + rnd.gauss(0, 0.05) for i in range(3))
        out.append((q, wm, fm))
        q = M.qnorm(M.qmul(q, M.qexp(w[0] * dt, w[1] * dt, w[2] * dt)))
    return out


def run(data, dt=0.02, sig=0.05):
    q0 = data[0][0]
    F = M.Att(q0, (0.0, 0.0, 0.0), 0.02, 0.02, 0.05)
    err = []
    for k, (qt, wm, fm) in enumerate(data):
        F.gravity(fm, sig)
        F.predict(wm, dt, 0.002 ** 2 * 0.005 * dt, 1e-10)
        qn = data[k + 1][0] if k + 1 < len(data) else qt
        d = M.qlog(M.qmul(qn, (F.q[0], -F.q[1], -F.q[2], -F.q[3])))
        err.append(d)
    return F, err


if __name__ == "__main__":
    bias = (0.01, -0.02, 0.03)
    for amp in (0.6, 0.05):
        F, err = run(simulate(bias=bias, tilt_amp=amp))
        e = err[-1]
        print(f"tilt_amp={amp}: bias est {tuple(round(x, 4) for x in F.bg)} true {bias}; final att err "
              f"tilt {math.degrees(math.hypot(e[0], e[1])):.3f} deg heading {math.degrees(e[2]):.3f} deg")
    F, err = run(simulate(bias=bias, tilt_amp=0.6))
    ok = all(abs(F.bg[i] - bias[i]) < 2e-3 for i in range(3))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
