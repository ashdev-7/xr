# EXPERIMENTS (append-only log)

Every number below was produced by a script on the code state of its entry. Benchmark: REAL IMU
(EuRoC MAV, TUM-VI room) degraded by the UNMODIFIED official `degrade_imu.py`, scored by
`tools/bench/score.py` (SE(3) Umeyama on all matched poses, as the FAQ states). Metrics in metres
unless noted: ATE, WP (pre-failure RMSE), BLIND (5 s after failure), DROP (burst dropouts + 1 s),
RPE_t (1 s, m), RPE_r (1 s, deg), ARE (deg). Scenarios: `asis` = script run on the EuRoC-order file
(gyro gets the accel-sized degradation), `aware` = columns presented in the script's expected order.

## 2026-11-21 — infrastructure
- Real data: TUM-VI room1-6 (IMU + mocap) via HTTP range reads of the tar; EuRoC via streamed
  zip-in-zip extraction from the ETH Research Collection (one GET per flight).
- `tools/bench/degrade_labels.py` reproduces the official degradation bit-identically (asserted)
  and returns per-row labels (spike / stuck / duplicate / failed). Used for evaluation only.
- Key facts measured from the official script: failure = last 5 s set to zeros (rows kept);
  dropouts = single rows (p = 0.06 s) + bursts of 5-50 rows; duplicates repeat the timestamp;
  stuck runs are exact repeats; every impairment scales with one severity s(t) = 0.02 + 0.98 (t/T)^1.7.

## 2026-11-21 — baselines (4 flights x 2 scenarios, seed 42)
| method | ATE | WP | BLIND | DROP | RPE_t | RPE_r | ARE |
|---|---|---|---|---|---|---|---|
| stand-still | 1.266 | 1.285 | 0.726 | 1.166 | 0.790 | 41.7 | 114.5 |
| naive double integration | 2769 | 2491 | 6025 | 2725 | 121 | 41.7 | 130.5 |
| legacy C++ v1 (previous session) | 1.180 | 1.196 | 0.722 | 1.097 | 0.685 | 1.45 | 8.4 |
| Python v0 (first port) | 1.204 | 1.221 | 0.723 | 1.114 | 0.711 | 9.03 | 36.4 |

## 2026-11-21 — spike detector (kept)
Per-channel rule (C++ heritage) deleted real fast handheld motion: on labelled data
(7 flights x 2 scen x 3 seeds) k=8 any-channel: TP 1119, FP 6062 (precision 0.16).
Signature rule "all 6 channels > 8 robust sigma": TP 960, FP 29, FN 166 (precision 0.97, recall 0.85).
Effect: RPE_r on TUM room2/4 15-19 deg -> ~3 deg. Mean over 14 variants: Python 1.078 vs C++ 1.069 vs still 1.179.

## 2026-11-21 — error budget with oracles (7 flights, aware, seed 42)
| configuration | ATE |
|---|---|
| clean IMU (max_level 0), old prior | 1.027 (still 1.179) |
| clean IMU, weaker prior (sig_v_h 1, sig_p_h 3, sig_a 0.1, ba_rw 0.01) | 0.766 |
| + ground-truth tilt | 0.722 |
| + ground-truth attitude | 0.598 |
Conclusion: the synthetic-data prior was far too strong for real IMU; attitude (tilt + heading) was the
largest remaining error source.

## 2026-11-21 — heading drift diagnosis (kept fixes)
- TUM-VI true gyro bias ~0.0004 rad/s (calibrated), but the filter estimated 0.006-0.013 rad/s:
  the old noise-ratio "jamming" scaling read motion intensity as jamming and let the bias wander.
- Noise estimator study (2nd-difference MAD vs injected noise): on TUM-VI it recovers the injected
  noise within ~5 %; on EuRoC rotor vibration is 5-12x the injected noise, so white-noise-based
  jamming estimation is impossible on drones.
- New jamming meter: severity from PACKET LOSS (missing rows per second), fit
  miss(s) = 0.0675 s + 0.03375 s^2 with s = L (t/T)^gamma. Estimates: L 0.92-1.0 (true 1.0) on drone
  and handheld, both scenarios; L = 0.03 on the clean variant. White-noise levels used as absolute values.
- Room4 heading RMS 13.4 -> 3.1 deg. Error-state RTS attitude smoother (replaces two-filter fusion):
  RPE_r 2.0 -> 1.55 deg on clean data.
- Degraded vs clean after the fix (aware, 7 flights): 0.712 vs 0.685; GT-attitude oracle 0.538 / 0.527.
  The jamming now costs ~0.03 m; attitude ~0.17 m.
- Tighter gyro-bias prior bg0_sig 0.05 -> 0.005: ATE 0.712 -> 0.656, ARE 15.5 -> 6.3 deg.

## 2026-11-21 — discretization (kept)
Prior pseudo-measurements were per knot, so the prior strength depended on knot_dt (0.2: 1.253,
0.1: 1.174, 0.05: 1.111). Scaled to continuous time: 0.646 / 0.656 / 0.650 (7 flights).

## 2026-11-21 — motion spectra (GT)
Share of position variance by period: MH_01 86 % > 60 s; V1_02 57 % in 20-60 s; TUM room2 85 % in 5-10 s.
Slow excursions of the machine hall are unobservable for any IMU-only method; goal there: never worse
than standing still.

## 2026-11-21 — rest detection (kept)
Detector gyro std < 0.04 rad/s, accel std < 1.0, | |f| - g | < 0.3 over 0.5 s, >= 1 s: 43/84 truly
still windows, 1 false positive (5 cm/s). Zero-velocity + zero-angular-rate updates; calm-window
search widened to 60 s (MH_01 rest starts at 20 s). MH_01 aware 4.475 -> 4.091 (still 4.323);
RPE_r MH_01 2.35 -> 0.35 deg. Mean 16 variants 1.174 -> 1.108.

## 2026-11-21 — blind forecast (kept: smoother prior forecast)
16 variants, BLIND: smoother forecast 1.237, hold 1.250, decaying velocity tau 0.5/1/2 s: 1.335/1.521/1.822.

## 2026-11-21 — engineering
- Closed-form KF/RTS and ESKF predict (outputs identical): 12.1 -> 6.1 ms per s of IMU.
- Single-pass ingest + lazy zipfile import: data heap 3.4 -> 2.2 MB (MH_01, 182 s), 5.4 ms/s.
- Compiling main.py itself costs 6.2 MB of traced heap (Python 3.13; 5.3 MB on 3.10), ~5 KB per line.
- Under tracemalloc pure-Python float arithmetic is traced: ~170 ms/s. Runtime is reported without a profiler.
- Contract suite (18 input variants) passes; any input (empty, 1-2 rows, garbage) yields one pose per row.

## 2026-11-21 — body-frame accelerometer bias (rejected)
EuRoC accel bias (GT) is constant 0.16 m/s^2 with 0.157 horizontal in body frame. Oracle removing it
before degradation: 3 EuRoC flights ATE 3.429 -> 3.412. Not worth a body-frame bias model.

## 2026-11-21 — platform signature
Median per-second accel noise (first half of flight): EuRoC 0.51-1.03 m/s^2, TUM-VI 0.027-0.029.
Tuning on a TUM-dominated TRAIN set tightened the prior and hurt EuRoC (MH_01 4.09 -> 4.25, V1_02 1.28 -> 1.43).

## 2026-11-21 — rotor-drag velocity measurement (kept)
Hypothesis: for a multirotor the specific force perpendicular to the thrust axis is rotor drag,
f_perp = -mu v_perp, so the accelerometer observes horizontal velocity (Leishman et al. 2014).
Check on GT (airborne segments, 2 s averaging): mu 0.18-0.20 1/s on MH_01, MH_02, V1_02; R^2 0.70-0.87;
implied velocity noise 0.17-0.25 m/s. Rotors off while hand-carried (vibration 0.03 m/s^2), on-ground
spin-up ~0.2, airborne 0.5-1.7 -> drag used when vibration > 0.4. Thrust axis = mean specific force
direction while airborne (EuRoC IMU is mounted ~20 deg tilted: u = (0.945, 0.001, -0.327)).
Result (seed 42, aware/asis), untuned drag_sig 1.2:
| flight | no drag | drag | stand-still |
|---|---|---|---|
| MH_01 aware/asis | 4.089 / 4.313 | 3.243 / 3.704 | 4.323 |
| MH_02 aware/asis | 4.357 / 4.595 | 3.807 / 3.942 | 4.665 |
| V1_02 aware/asis | 1.283 / 1.314 | 1.175 / 1.187 | 1.778 |
Mean ATE 3.325 -> 2.843; BLIND 4.043 -> 3.504 (V1_02 blind worse: 1.03 -> 1.35, to investigate).

## 2026-11-21 — rotorcraft blind window (kept: hold for rotorcraft)
GT: EuRoC flights decelerate/land at the end (speed at failure 0.17-0.72 m/s, mean in the blind window
0.12-0.23 m/s; z drops 0.3-0.4 m). With drag on, 4 EuRoC flights x 2 scen: BLIND smoother forecast 3.342,
hold 3.227, decaying velocity 0.5 s 3.385. Rotorcraft preset: blind_tau = 0 (hold).

## 2026-11-21 — rotorcraft position prior probe (aware, MH_01-03 + V1_02)
sig_p_h 3 -> 10: ATE 2.738 -> 2.594 (BLIND 3.251 -> 3.149). Weaker still (10 + sig_c 0.5, 30 + sig_c 1):
BLIND 4.2-4.7 (end-of-flight drift goes into the held position). Tighter drag (drag_sig 0.8): worse (3.378).
Diagnosis (MH_01): estimated excursion ~3 m vs true ~10 m: the workspace prior shrinks the trajectory.

## 2026-11-21 — drag error spectrum and Gauss-Markov drag-error state (rejected)
Drag-velocity residual vs GT (airborne, 10 Hz): white ~0.5-0.65 m/s; after 5 s averaging 0.12-0.30 m/s
(2-3x the white expectation) -> correlated component; mean offset 0.1-0.2 m/s.
5-state KF/RTS with a Gauss-Markov drag error e (z = v + e + white), 5 EuRoC flights aware, sig_p_h 10:
white-only 2.936; +GM (0.2 m/s, 5 s) 3.309; (0.2, 5 s, white 0.7) 3.227; (0.3, 10 s, white 0.7) 3.461.
Rejected: the long-term mean of the drag velocity is exactly the information that tracks slow excursions;
a correlated-error state explains it away. Also 1.5-2x slower.

## 2026-11-21 — scenario-correct bias random walks (rejected, small)
In `asis` the gyro gets the accel-sized bias RW (0.01 s) and the accel the gyro-sized one (0.0006 s).
Oracle with correct values, 5 EuRoC flights asis: 3.290 -> 3.239 with bg_k 0.01 (-1.5 %); ba_k 0.001 hurts
(3.436): the accel-bias state also absorbs tilt and other slow errors. Not worth a scenario detector.

## 2026-11-21 — handheld class tuning (kept)
Coordinate descent on TUM-VI room1/3/5 (2 scen x seeds 42, 1): J_train 0.5879 -> 0.5644
(sig_v_h 0.7 -> 1.96, ba_k 0.01 -> 0.028, tilt_sig 0.5 -> 0.25). Held-out TUM room2/4/6 (2 scen x 3 seeds):
J_test 0.6788 -> 0.6738, ATE 0.551 -> 0.532, BLIND 0.602 -> 0.611.

## 2026-11-21 — scorer vs evo; blind-window attitude (kept, small)
Cross-check: our ATE equals evo_ape -a to 4 decimals (room2 0.4747/0.474659, MH_01 4.0890/4.089175,
V1_02 1.2830/1.282939). evo_rpe has no "seconds" unit (frames/deg/rad/m); default is 1 frame. Over all
rows (incl. the blind window) RPE rotation is dominated by the blind window when the attitude is held
(room2 blind-window absolute rotation error 45 deg). Scorer now also reports rpe_t_all / rpe_r_all
(200 frames, all rows). Blind attitude = last body rate decaying with att_tau, TRAIN flights (5 x 2 scen):
| att_tau | rpe_r (1 s, starts before failure) | rpe_r_all | ARE |
|---|---|---|---|
| 0 (hold) | 1.689 | 3.907 | 16.24 |
| 0.3 | 1.445 | 3.886 | 16.62 |
| 0.7 | 1.372 | 4.083 | 17.48 |
| 1.5 | 1.392 | 4.441 | 19.09 |
Kept 0.3 s.

## 2026-11-21 — severity-scaled drag-velocity bias random walk (rejected)
Hypothesis: late-flight accel bias drift (aware) corrupts the drag velocity; a bias RW growing with s(t)
absorbs it without explaining away early slow motion. MH_01/03/05 x 2 scen, seed 42:
white-only ATE 3.872 BLIND 3.183; RW 0.05 s: 3.869 / 4.167; 0.1 s: 3.990 / 4.345; 0.2 s: 4.170 / 4.390.
Rejected: blind error rises sharply. The white-noise drag measurement remains best.

## 2026-11-21 — code review fixes (no accuracy change: 22 variants, seed 42, mean ATE 1.765, 0 contract issues)
Guard for out-of-order / far-future timestamps (would have grown arrays without bound); t_end from
accepted rows; utf-8-sig for folder inputs; dead code removed.

## 2026-11-21 — return-to-launch prior (rejected on principle + small gain)
GT: every EuRoC flight ends within 0.01-0.26 m of its start (spread 1.8-6.9 m); TUM-VI rooms 0.5-1.1 m
(spread ~1 m). Dev-only prototype (constant p0 state, end tied to start with sigma_loop), 6 EuRoC flights
x 2 scen, seed 42, sig_p_h 10: off 3.345 (BLIND 2.351); sigma 1 m: 3.214 (2.318); 0.3 m: 3.197 (2.383).
Rejected: a -4.4 % gain does not justify a dataset-behaviour assumption (the task describes an aircraft
flying to a target) that would fail badly on a non-returning flight. Not in src/main.py.

## 2026-11-21 — rotorcraft class tuning, round 1 (kept)
TRAIN MH_01/03/05 (2 scen x seeds 42, 1), start sig_p_h 10: J_train 0.7590 -> 0.6389
(sig_v_h 2, sig_a 0.2, ba_rw 0.02, ba_k 0.02, sig_c 0.1, tilt_sig 0.25, tilt_k 2, sig_gap 6).
Held-out MH_02, MH_04, V1_02, V1_01 (2 scen x 3 seeds): J 0.8647 -> 0.7617, ATE 2.887 -> 2.651,
BLIND 2.265 -> 1.867. Per flight ATE: MH_02 3.878 -> 3.937, MH_04 4.310 -> 3.729, V1_02 1.199 -> 1.341,
V1_01 2.162 -> 1.598.
