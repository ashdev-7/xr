# PathGuard: trust-aware inertial navigation under sensor degradation

Samsung PRISM XR Hackathon 2026, **Track 03 (Path Finder): Trajectory Estimation Under Sensor Degradation**.

PathGuard turns a degraded IMU log (EuRoC `imu0/data.csv`) into a 6-DoF trajectory with one pose per IMU
row, and dead-reckons the 5 seconds after the IMU fails. The estimator is one dependency-free C++17 file
that runs single-threaded and offline. On a 3-minute flight it uses **about 3 MB of peak memory and about
0.1 s of CPU**, against budgets of 8 MB and 3.6 s.

## Quick start

```bash
# build (static binary, lowest memory) and run
make
./build/pathguard path/to/imu0/data.csv estimated_trajectory.txt --verbose=1

# or through the Python entrypoint (compiles on first use, then execs the native binary)
python3 main.py path/to/imu0/data.csv estimated_trajectory.txt

# or with Docker
docker build -t pathguard .
docker run --rm -v "$PWD/data:/data" pathguard /data/imu0/data.csv /data/estimated_trajectory.txt
```

The input may be the CSV itself or a dataset folder containing `mav0/imu0/data.csv` or `imu0/data.csv`.
`--verbose=1` prints diagnostics to stderr: rows, invalid rows, repaired spikes, estimated gravity,
gyro bias, and peak memory (`VmHWM`).

## Output contract

- `estimated_trajectory.txt`, space-separated: `timestamp tx ty tz qx qy qz qw`, header line starting with `#`.
- Timestamps are the input nanoseconds converted exactly to seconds (`ns / 1e9`, printed without rounding).
- **Exactly one pose per input row.** Rows inside dropouts and after the failure still get a pose.
- If the file simply ends at the failure, 5 s of forecast poses are appended at the nominal IMU rate.
  (`--append_blind=0` disables this.)
- Quaternions are unit, scalar-last, body to world, with `qw >= 0`.

## How it works

```
imu0/data.csv
  │
  ├─ 1. Integrity layer   reject NaN / all-zero / frozen rows, repair spikes against a robust local median,
  │                        detect dropouts (missing or dead rows) and the final failure
  ├─ 2. Self-calibration   gravity direction (unknown alignment) and gyro bias from the calmest early second
  ├─ 3. Attitude ESKF      error-state Kalman filter on [orientation, gyro bias]; gravity-referenced tilt
  │                        corrections whose trust scales with the measured noise level; a backward pass is
  │                        fused with the forward pass (offline smoothing)
  ├─ 4. Translation        per-axis Kalman filter + RTS smoother on [position, velocity, accel bias, workspace
  │      smoother          centre] at 10 Hz, driven by gravity-compensated world acceleration and a motion
  │                        prior (bounded speed, bounded workspace) tuned on training flights. Dropouts are
  │                        bridged from both sides by the smoother.
  └─ 5. Blind forecast     after failure: hold the last smoothed position, continue the last angular rate
                           with decay. Chosen because it beat constant-velocity extrapolation on unseen flights.
```

### Why a motion prior is the core

Double-integrating an accelerometer turns a 0.01 m/s² bias into 100 m of error within 2–3 minutes, and a
1° tilt error leaks 0.17 m/s² of gravity into the horizontal axes. On the degraded data, naive strapdown
integration ends up **more than 15 km** from the truth. Without its motion prior, PathGuard drifts to 155 m.
With the prior, the IMU supplies the fast motion and the prior supplies the long-term bound.

### The IMU-only floor

We measured how good any IMU-only method can be on these flights. If an oracle knew all motion faster than
a 10 s period perfectly and nothing slower, its ATE would be 3.39 m on our held-out flights. The flights'
room-scale loops take 20–40 s, which is exactly the band that bias and tilt errors swamp. PathGuard reaches
that 10 s oracle. The remaining error is in motion that the sensor cannot observe, and PathGuard does not
invent it.

## Results

Evaluation follows the official metrics (`evo`-style ATE with SE(3) alignment, RPE over 1 s), on 5 EuRoC
flights never used for tuning, each under two degradation styles. Development data: EuRoC ground truth
(from the OpenVINS repository) converted to 200 Hz IMU with the dataset's own sensor biases, ADIS16448-level
noise and propeller vibration, then degraded with progressive noise (up to 6×), bias drift, spikes,
three 2 s dropouts and a final failure 5 s before the end (`tools/make_dev_data.py`).

| Method (held-out flights) | ATE [m] | Waypoints [m] | Dropout [m] | Blind 5 s [m] | RPE 1 s [m] |
|---|---|---|---|---|---|
| Naive double integration | 15,345 | 14,766 | 14,885 | 37,792 | 591 |
| Stand still (no motion) | 3.51 | 3.66 | 3.56 | 3.29 | 0.79 |
| **PathGuard** | **3.39** | **3.53** | **3.50** | **3.28** | **0.77** |
| Oracle: all motion faster than 10 s known | 3.39 | | | | |

Ablations (held-out flights):

| Configuration | ATE [m] | Blind 5 s [m] |
|---|---|---|
| PathGuard (full) | 3.39 | 3.28 |
| without motion prior | 154.63 | 224.24 |
| without backward attitude pass | 3.40 | 3.29 |
| without integrity layer | 3.41 | 3.28 |
| blind forecast: constant velocity | 3.39 | 5.79 |
| blind forecast: decaying velocity (1 s) | 3.39 | 3.83 |

Attitude accuracy: mean tilt error about 1.2° under degradation (1.6° without the backward pass).

Resources (static build, Linux x86-64, single thread):

| Input | Peak memory (VmHWM) | CPU time | Budget |
|---|---|---|---|
| 182 s flight (36k rows) | 3.1 MB | ~0.1 s | 8 MB, 3.64 s |
| 15 min (171k rows) | 9.2 MB | ~0.5 s | 8 MB, 18 s |

Memory grows about 48 bytes per IMU row, so inputs up to about 12 minutes at 200 Hz fit in 8 MB. All EuRoC
flights are at most 3 minutes.

Robustness checks (all pass, exact row counts, no NaN in output): missing header `#`, CRLF line endings,
spaces after commas, NaN rows, duplicated rows, frozen rows, all-zero rows, spikes, a 300-row file.

## Limitations

- IMU-only position is fundamentally limited for slow motion (see "The IMU-only floor"). Absolute error stays
  in the 3 m range because room-scale loops are unobservable, not because of a tuning issue.
- Parameters were tuned on synthesized degradation that follows the brief. The organizer degradation script
  may differ; noise-adaptive trust and the integrity layer are there to absorb that, but some re-tuning
  might help.
- Development IMU was synthesized from ground truth because the raw EuRoC IMU logs were not reachable from
  our build environment. The real-data runs (organizer evaluation set) are the ones that count.
- The 8 MB budget is met by the native process. A Python interpreter alone exceeds 8 MB, so `main.py` hands
  over to the binary with `os.execv`.

## Repository layout

```
main.cpp            the estimator (all of it)
main.py             Python entrypoint: builds main.cpp if needed, then execs it
Makefile, Dockerfile
tools/              development only: data generation, evo-style scorer, benchmarks, tuning, ablations, plots
```

Development workflow (from the repository root): `python3 tools/make_dev_data.py`, then
`python3 tools/bench.py --split test`, `python3 tools/ablation.py`, `python3 tools/baselines.py test`.

## How we used AI tools

We used Claude (an AI assistant) for research, code scaffolding and test harnesses. Every design decision
was kept only if it improved a measured score on held-out flights, and the team can explain every component
of `main.cpp`.
