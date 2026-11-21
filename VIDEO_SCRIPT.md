# PathGuard: pipeline walkthrough video (target 2:40, hard limit 3:00)

Record a screen plus voice. Show the slide or terminal named in each block. Speak at a calm pace; the word
counts below fit the timings.

---

**0:00–0:20 · Hook** (slide 3, error-over-time chart)

> "An aircraft is flying through checkpoints with GPS jammed. All it has is an IMU, and the jamming is
> getting worse until the sensor dies. If you do the obvious thing and integrate the accelerometer twice,
> you end up fifteen kilometres from where you really are. This is PathGuard, our answer for Track 3."

**0:20–0:45 · The problem** (slide 2)

> "The input is a raw IMU log: rotation rate and acceleration, two hundred times a second, with rising
> noise, drifting bias, two-second dropouts, and finally a dead sensor. We must output a full 6-DoF pose for
> every sample, predict five seconds after the failure, and stay under eight megabytes and twenty
> milliseconds per second of data, single-threaded."

**0:45–1:25 · Live run** (terminal)

```bash
make
./build/pathguard data/dev/V2_01_easy/zeros estimated_trajectory.txt --verbose=1
head -3 estimated_trajectory.txt
python3 tools/bench.py --split test
```

> "One C++ file, no libraries. Here it runs on a flight it never saw during tuning. The log line shows what
> the integrity layer found: dead rows, repaired spikes, the estimated gravity, and the peak memory:
> about three megabytes. It finishes in about a tenth of a second for a three-minute flight. The output has
> exactly one pose per input row in the official format."

**1:25–1:55 · How it works** (slide 4)

> "Five stages. First, reject and repair bad samples. Second, calibrate gravity and gyro bias from the data
> itself, because the alignment is unknown. Third, an error-state Kalman filter for attitude, run forward
> and backward and fused, so we use the whole log. Fourth, a Kalman smoother for position with a motion
> prior learned from training flights. That is what bridges the dropouts from both sides. Fifth, after the
> failure, we hold the last smoothed position, because on unseen flights that beat extrapolating a velocity
> we cannot trust."

**1:55–2:30 · The insight and results** (slide 5, then slide 6)

> "Here is the most important result. We asked: what is the best any IMU-only method could do on these
> flights? We built an oracle that knows every movement faster than ten seconds, perfectly. It scores 3.39
> metres. PathGuard scores 3.39 metres. The remaining error is slow, room-scale motion that no accelerometer
> can see through this much drift, and PathGuard does not invent it. Naive integration: fifteen thousand
> metres."

**2:30–2:45 · Close** (slide 8)

> "Three megabytes, under a millisecond per second of data, no crashes on any malformed input, and every
> design choice backed by a measured ablation. PathGuard: navigation that knows how much to trust its own
> sensor. Thank you."

---

Checklist before recording
- Run `make clean && make` once so the live build is real, not cached.
- Use a terminal font of at least 18 pt so the log is readable.
- Record twice; keep the cleaner take. Upload to YouTube (unlisted) or Drive with link sharing on.
