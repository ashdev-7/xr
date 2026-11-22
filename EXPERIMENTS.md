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

## 2026-11-21 — "scale": three meanings, measured
1. Sensor scale-factor drift (official script, 0.0015 s per sqrt(s) on both sensors). Oracle with
   scale_rw_max = 0 (same seed/noise otherwise), 9 flights seed 42: aware ATE 1.780 (with) vs 1.786 (without);
   asis 1.880 vs 1.895. No measurable cost: the accel effect is mostly along gravity and is absorbed by the
   vertical bias random walk; the gyro effect is ~1 % of net rotation, bounded by tilt/rest updates.
2. Trajectory amplitude (SE(3) evaluation cannot undo a wrong size). Sim(3) scale of GT vs estimate (aware):
   MH_01 1.81, MH_02 2.13, MH_04 1.64 (machine hall: we shrink ~2x); V1_01 0.75, V1_02 0.73, V1_03 0.92
   (Vicon room: slight overshoot); TUM room1/2/4 1.05/1.11/1.44. Oracle amplitude correction: MH ATE -10..-25 %.
   Prior-size grid (aware, seed 42), ATE:
   | flight | p3/v1 | p3/v2 | p3/v4 | p10/v1 | p10/v2 | p10/v4 | p30/v1 | p30/v2 | p30/v4 | p100/v1 | p100/v2 | p100/v4 |
   |---|---|---|---|---|---|---|---|---|---|---|---|---|
   | MH_01 | 3.78 | 3.72 | 3.70 | 3.39 | 3.23 | 3.18 | 2.76 | 2.54 | 2.65 | 3.23 | 4.64 | 5.70 |
   | MH_02 | 4.26 | 4.20 | 4.18 | 3.93 | 3.73 | 3.66 | 3.50 | 3.01 | 2.86 | 3.35 | 2.85 | 2.87 |
   | MH_04 | 5.73 | 5.56 | 5.51 | 4.53 | 3.65 | 3.36 | 3.97 | 2.81 | 2.78 | 4.00 | 3.52 | 4.11 |
   | V1_01 | 1.55 | 1.52 | 1.51 | 1.48 | 1.48 | 1.51 | 2.09 | 2.58 | 2.84 | 3.55 | 5.85 | 7.04 |
   | V1_02 | 1.21 | 1.13 | 1.11 | 1.25 | 1.34 | 1.43 | 1.50 | 2.10 | 2.48 | 1.60 | 2.43 | 2.97 |
   | V1_03 | 1.29 | 1.26 | 1.25 | 1.19 | 1.16 | 1.17 | 1.41 | 1.73 | 1.94 | 1.72 | 2.61 | 3.15 |
   Per-flight oracle mean 1.99 vs tuned p10/v2 2.43 (best single setting p10/v4 2.385).
   Per-flight selection from data FAILED: drag-velocity roam spread, speed std and persistence overlap
   (V1_02 vs MH_02/MH_03); the marginal likelihood of the drag measurements increases monotonically with the
   prior size on every flight (plateau from p30), i.e. it would pick the hall setting everywhere.
   Conclusion: amplitude is not identifiable from the IMU alone; keep the tuned compromise. drag_loglik and
   roam statistics are kept as verbose diagnostics only.
3. Unit scale (g vs m/s^2, s/ms/us/ns): auto-detected, covered by tests/test_contract.py.

## 2026-11-21 — thrust-scaled drag coefficient (not adopted, negligible)
mu(t) = mu0 (f_thrust / mean)^e (literature: drag ~ rotor speed). 9 EuRoC flights x 2 scen, seed 42:
e = 0: ATE 2.600 BLIND 1.961; e = 0.5: 2.598 / 1.954; e = 1: 2.595 / 1.948. Kept e = 0 (simplicity).
Held-out V2_01 (seed 42): ATE aware 2.053 / asis 2.219 vs stand-still 2.295.

## 2026-11-21 — rotorcraft class tuning, round 2 (kept)
TRAIN MH_01/03/05 + V1_01 + V2_02 (2 scen x seeds 42, 1): J_train 0.7033 -> 0.6535
(sig_v_h 2 -> 4, sig_a 0.2 -> 0.4, ba_rw 0.02 -> 0.04, ba_k 0.02 -> 0.04, sig_c 0.1 -> 0.05).
Held-out MH_02, MH_04, V1_02, V1_03, V2_01 (2 scen x 3 seeds): J 0.8105 -> 0.8093, ATE 2.485 -> 2.466,
BLIND 2.036 -> 1.944. Per flight: MH_02 3.94 -> 4.02, MH_04 3.73 -> 3.51, V1_02 1.34 -> 1.43,
V1_03 1.30 -> 1.23, V2_01 2.12 -> 2.14.

## 2026-11-21 — decimated drag updates (rejected; code reverted)
Literature suggestion for correlated measurement errors: one update per window on the window mean.
TRAIN rotor flights x 2 scen, seed 42: every knot 2.464 (BLIND 1.746); every 10 knots, variance x1 3.252,
x0.3 2.908, x0.1 2.482 (1.741, equivalent to per-knot); every 20, x0.2 3.025. Fourth independent experiment
(after GM state, drag-bias RW, tighter drag_sig) showing that the slow content of the drag velocity is signal.

## 2026-11-21 — FINAL held-out results (tools/bench/report.py, TEST flights, 2 scen x 3 seeds)
Rotorcraft (MH_02, MH_04, V1_02, V1_03, V2_01, V2_03): ATE 2.374 (stand-still 3.197, legacy C++ 3.167,
naive 18645); BLIND 2.209 (2.885); RPE 1 s 0.354 m / 0.884 deg.
Handheld (TUM room2/4/6): ATE 0.532 (stand-still 1.096, legacy C++ 0.987, naive 644); BLIND 0.611 (0.621).
Ablations (rotor / handheld ATE): no drag 2.965 / -, no rest 2.413 / -, no spike repair 2.374 / 0.518,
no jamming adaptation 2.388 / 0.509 (BLIND 2.473 / 0.676), filter only 2.379 / 0.536, rotor blind smoother
forecast 2.371 (BLIND 2.415). Resources: 6.86 / 6.19 ms per s; estimator heap 1.85 / 1.73 MB; 7.82 MB incl. compile.
Follow-up on TRAIN handheld (room1/3/5, 2 scen x 3 seeds): repair=0 J 0.5523 -> 0.5495 (-0.5 %), adapt=0 0.6161.
Decision: keep spike repair (tiny cost on this script's handheld spikes, protection against stronger jamming);
keep adaptation (clearly better on TRAIN; the TEST ATE dip is within noise and its BLIND is worse).

## 2026-11-21 — systematic pass over the literature's ranked gaps (docs/literature_review.md)
Per-flight drag fit vs GT (1 s averaging, vibration-gated airborne): mu axis1/axis2 MH 0.16-0.20/0.20-0.22,
V-room 0.09-0.20/0.17-0.22; quadratic term changes sign between flights and adds <= 0.01 R^2.
-> The review's hypothesis "mu differs by environment and explains the hall/room amplitude error" is refuted.
Anisotropic drag (mu 0.17 / 0.21, fitted on TRAIN): TRAIN 2 scen x 2 seeds ATE 2.488 -> 2.548 (rejected; mu acts
as an amplitude knob already balanced by tuning).
Blind window, masked backtests: each TRAIN flight cut at 45/60/75/90 % and re-degraded with the official script
(failure at maximum jamming at every cut), 8 windows per flight. BLIND [m]:
| blind_tau | drone (40 windows) | handheld (24 windows) |
|---|---|---|
| 0 (hold) | 3.195 | 1.158 |
| 0.3 | 3.196 | 1.102 |
| 0.7 | 3.197 | 1.134 |
| smoother forecast | 3.203 | 1.110 |
The forecast rule barely matters; the blind error is the position error at the moment of failure.
Kept: handheld blind_tau 0.3 (OU-optimal decay); drones keep hold.
Accel bias coupled into the drag measurement (z = v - b/mu, Abeywardena 2016 / HDVIO): TRAIN ATE 2.488 -> 3.534
(weight 1) / 3.253 (0.5); backtests 2.896 -> 3.472. Rejected: in the decoupled design the world-frame bias state
also absorbs tilt-induced gravity leakage, which the drag measurement does not contain.
Ceiling of any attitude improvement (tight coupling, gyro calibration, InEKF refinements): drone TRAIN 2 scen,
GT attitude oracle 2.464 -> 2.445 (-0.8 %); handheld earlier -5 %. Not worth a full-state rebuild.
Other ceilings from ablations (held-out): spike handling 0 % (no repair: same ATE), rest detection 1.6 %,
blind forecast <= 1 %, jamming adaptation 0.6 % ATE / 11 % BLIND (kept). The only large remaining prize is
trajectory amplitude (oracle -18 %), which no IMU-only statistic tried so far identifies.

## 2026-11-22 — handheld blind decay reverted
Final report with handheld blind_tau 0.3 (adopted from TRAIN backtests, +0.7 %): held-out handheld BLIND
0.611 -> 0.652 (ATE 0.532 -> 0.535). A small TRAIN gain reversing on held-out flights is noise; reverted to the
smoother forecast. README/deck keep the previous final report (unchanged code path).

## 2026-11-22 — frequency-resolved amplitude diagnostic (VIO / GP review tracks)
Transfer gain (cross-spectrum est.gt / gt power) on held-out flights, aware seed 42, by period band:
2-20 s 0.8-1.0; 20-60 s 0.5-0.8; > 60 s 0.1-0.4 (MH_02: 0.12 on the band holding 79 % of its motion).
Confirms the high-pass hypothesis: fast motion is reproduced at full size, slow excursions are filtered out.

## 2026-11-22 — Student-t (scale-mixture) workspace prior (rejected; code not kept)
IRLS on the horizontal p - c residual, w = (nu+2)/(nu+d^2), 3 passes. Drone TRAIN 2 scen x 2 seeds ATE:
Gaussian 2.488; nu 10: 2.540; nu 3: 2.641; nu 1: 2.827. Releasing large excursions also releases slow drift.

## 2026-11-22 — speed-measurement family (Solin 2018, Cortes 2018, RIDI/RoNIN/AirIO ridge analogue)
Oracle: true 1 s horizontal speed as a velocity measurement along the pass-1 direction (dev-only patch, sigma
0.3 m/s), TRAIN aware seed 42, ATE: MH_01 3.460 -> 2.586, MH_03 1.929 -> 1.394, V1_01 1.368 -> 1.122,
V2_02 1.378 -> 1.147, room1 0.317 -> 0.297, room3 0.510 -> 0.324. Large prize if speed were known.
Constant speed (Solin-style; drone 0.5, handheld 0.85 m/s from TRAIN GT), sigma 0.3 / 0.6: MH_01 3.20 / 3.30,
MH_03 3.14 / 2.70, V1_01 1.61 / 1.43, V2_02 1.53 / 1.46, room1 0.69 / 0.44, room3 1.04 / 0.68 -> rejected.
Learned speed (ridge on 8 heading-independent 1 s features of the degraded IMU, leave-one-flight-out, 4
degradation variants per flight; criteria fixed before running: r >= 0.6, RMSE <= 35 % / 25 % of mean):
drone mean r 0.22 (3 of 5 flights worse than a constant), handheld mean r 0.47 (RMSE 41-54 %) -> rejected.
The literature trains speed models on 30 min - 60 h; 5 drone / 3 handheld flights are not enough.

## 2026-11-22 — full-text review follow-ups (reports/IMU odometry full text review.md, plan items 1 and 6)
Per-flight GT-fitted drag coefficient (ceiling for any online mu estimator), aware seed 42, ATE:
MH_01 3.460 -> 3.511, MH_03 1.929 -> 2.001, MH_05 3.751 -> 3.883, V1_01 1.368 -> 1.415, V2_02 1.378 -> 1.375.
Negative ceiling: online mu estimation dropped.
Component switch-off (drone TRAIN, 2 scen, seed 42), ATE / BLIND: baseline 2.464 / 1.746; bias RW x0.2
2.517 / 1.959; centre RW 0.01 2.478; centre RW 0.5 4.649 / 7.694; velocity prior off (sig_v_h 20) 2.450 / 1.725;
workspace prior off (sig_p_h 100, sig_c 0.5) 5.914 / 10.381. The high-pass is the workspace/centre prior, and
it is the same component that stops drift: slow motion and drift are not separable from the IMU here.
Not run: noisy-speed oracle (moot after the learned speed model reached r = 0.22 / 0.47).

## 2026-11-22 — 2-second signal losses (deck: "2-second signal loss"; dropout tier 10 pts + T3 bonus) (kept)
Variants: official degradation (aware, seed 42) + three 2 s gaps (rows removed, FAQ representation) at 30/50/70 %
of each flight (tools/bench/make_gaps.py). Before the fix, held-out 9 flights: ATE 2.623 vs stand-still 2.491
(worse than standing still), DROP 2.705, RPE_r 8.5 deg. Cause: attitude uncertainty inside gaps grew only with
10x gyro noise, so after a 2 s gap the filter trusted a wrong attitude (tilt errors 20-54 deg after gaps).
Fix 1 (gap_alpha = 2 rad/s^2): rotation variance per empty bin (alpha L dt)^2, total ~alpha^2 L^3 (~2 deg for
0.25 s, ~0.8 rad for 2 s). Held-out with gaps: ATE 2.623 -> 1.961, DROP 2.705 -> 2.009, RPE_r 8.5 -> 3.9 deg.
Official-only data unchanged (1.729 -> 1.729).
Fix 2: separate accel uncertainty inside gaps (sig_gap_in) from the blind window (sig_gap). Chosen on TRAIN:
with gaps ATE 2.092 -> 2.026, DROP 2.101 -> 1.999; official ATE 1.696 -> 1.693, BLIND unchanged 1.315.
Held-out confirmation: with gaps ATE 1.917, DROP 1.974 (stand-still 2.491 / 2.525); official 1.726.

## 2026-11-22 — efficiency (40 of 100 points: peak memory 20, runtime 20)
measure.py fixed: timing came from the tracemalloc run (~30x slowdown, reported 187 ms/s); now memory from one
traced run, time = median CPU of 3 untraced runs.
25 Hz attitude bins (bin_dt 0.04, gravity update every bin) instead of 50 Hz / every 2nd: MH_02 6.04 -> 4.59 ms/s,
estimator heap 1.83 -> 1.31 MB. Accuracy TRAIN official ATE 1.693 -> 1.698, held-out 1.726 -> 1.726 (blind
1.715 -> 1.729; with 2 s gaps 1.917 -> 1.921). Kept.
Ingest micro-optimizations (unrolled 0.5 s moments and second differences, no string replace when the row has no
whitespace): output bit-identical on MH_02 aware and room4 asis; ~4.6-5.1 ms/s (run-to-run noise +-0.3).
Total traced peak including CPython's compile of main.py: 7.78 MB, of which compile ~6.8 MB (transient, scales
with file size; half the file 2.6 MB). Not changed: splitting into modules would cut it to ~2.5 MB only if the
organizers trace compilation, and the FAQ says data structures/buffers are measured.

## 2026-11-22 — final refit on all public flights (objective = official 30/20/10 weights)
tune.py --cls rotor_final (11 EuRoC) / handheld_final (6 TUM-VI rooms), seeds 42 and 1, 2 rounds. Flat optimum:
rotor J 0.7710 -> 0.7681 (only sig_v_h 4 -> 2); handheld J 0.6103 -> 0.6076 (only sig_a 0.1 -> 0.05).
Checked on unseen seed 2 and on the 2 s-gap variants: rotor sig_v_h 2 worse (seed 2 ATE 2.472 -> 2.514, gaps
2.611 -> 2.638) -> rejected (in-sample noise); handheld sig_a 0.05 better on seed 2 (0.461 -> 0.457), gaps
neutral (0.802 -> 0.802) -> adopted.

## 2026-11-22 — correction to the efficiency entry above
The 1.83 -> 1.31 MB and 6.04 -> 4.59 ms/s figures came from a second run inside a warm process. Cold processes
(how a grader runs us), MH_02 aware s42: estimator heap 1.86 -> 1.64 MB, CPU 7.7 -> 5.8-6.2 ms/s. The README's
resource table (report.py) uses cold processes and is the number to quote.

## 2026-11-22 — blind-window margin (20 pts). Handheld blind is only 3 % below stand-still (0.60 vs 0.62 held-out)
Diagnosis (tools/bench/errwin.py, error after evo alignment by window relative to the failure F):
official ends, 6 rooms x 4 variants: PG first half 0.42, F-15..F-5 0.56, F-5..F-1 0.72, F-1..F 0.68, blind 0.68/0.57;
stand-still F-5..F-1 0.70, blind 0.77/0.60. The estimate degrades in the last seconds (one-sided smoother end
effect at maximum jamming); at the failure PG is no better than the trajectory centre. Drones: no end effect
(F-5..F-1 1.72 vs first half 2.50). Blind attitude error (23 deg tilt, 86 deg heading handheld) is the true
rotation during 5 s without data; it does not enter the translation ATE.
Rejected 1: shrink the blind forecast toward the mean estimated position (TRAIN backtests, 40 drone / 24 handheld
windows): handheld best = no shrink (1.103); drone best 3.221 -> 3.127 (-3 %) only with a position jump at F.
Rejected 2: zero-velocity prior on the blind knots (smoother sees "motion settles"), handheld TRAIN official:
blind 0.611 -> 0.635 / 0.693 / 0.778 / 0.849 for sigma 0.5 / 0.2 / 0.1 / 0.05 m/s.

## 2026-11-22 — worst-case memory: stage files compiled one at a time (kept)
If memory is traced from interpreter start (strictest harness), the single 1,350-line main.py peaked at 7.86 MB
on Python 3.13 (6.90 on 3.10): CPython parses a whole file at once (~7 MB transient), the data heap is 1.6 MB.
Change (tools/split_main.py): main.py = entry point, parameters, input reader, stand-still fallback + loader that
compiles pg_ingest / pg_math / pg_attitude / pg_translation / pg_output one at a time into main.py's namespace
(by path, so "python -I" works; candidates: __file__ dir, cwd, cwd/src). Output bit-identical on 7 inputs (drone,
handheld, 2 s gaps, backtest cut, sample) and on Python 3.10; identical under 6 launch modes (absolute path from
another cwd with -I, cd src, runpy, exec without __file__ from repo or src, import). main.py alone: stand-still
output with one pose per row (29480/29480) and a stderr message. Contract 18/18, ESKF test pass.
Worst-case traced peak: 3.13: 7.86 -> 2.91 (MH_02) / 3.15 MB (room2); 3.10: 6.90 -> 2.25 / 2.18 MB.
Largest single-stage parse 1.95 MB (3.13).

## 2026-11-22 — runtime: inlined timestamp parsing in both passes, tuple unpack in the spike test (kept)
Output bit-identical on the 7 reference inputs; contract 18/18. measure.py (fresh process, everything incl.
loading the stages, median of 3): MH_02 4.11 ms/s, room2 4.45 ms/s (was ~5.8-6.2); worst-case traced peak
2.99 / 2.78 MB. Floor for any pure-Python reader (parse the CSV twice + format the output): ~0.9 ms/s.

## 2026-11-22 — correction: main.py must be standalone (FAQ: "submit a standalone Python script (main.py)")
The stage-file split above would degrade to a stand-still trajectory if the evaluation copies only main.py.
Replaced by tools/build_main.py: main.py is one standalone file carrying the five stages (src/stages/*.py) as
embedded raw strings, compiled one at a time into its namespace. tests/test_build.py (also in CI) fails if main.py
drifts from the stages. Bit-identical on the 7 reference inputs; main.py alone in another folder, Python 3.10, -I:
bit-identical. Worst-case traced peak 2.96 / 3.23 MB (3.13), 2.30 / 2.26 MB (3.10); 3.9 ms per s of IMU.
Bundle layout fixed to the template: estimated_trajectory.txt, metadata.json and src/ at the zip root.
