# Track 03 (Path Finder): problem statement and evaluation, as given by the organizers

Transcribed from the organizers' Track 03 slides.

## About the problem
Core motivation: sensor jamming and progressive degradation. In GPS-denied and hostile environments, an
aeroplane navigates through waypoints capturing reconnaissance images. As it proceeds, enemy jamming causes
escalating IMU noise, bias drift, and frame dropouts until the IMU sensor fails entirely.
- Drift and frame dropouts: double integration explodes quadratically under sensor noise, bias drift, and
  2-second signal loss.
- Post-failure prediction: after the final checkpoint the IMU dies; the aircraft must dead-reckon its 3D path
  for 5 seconds.

Challenge abstract
- Estimate the 6-DoF trajectory (position + orientation) through all mission checkpoints using only degraded IMU
  streams.
- After complete IMU failure, dead-reckon the 5-second trajectory to strike the target.
- Budgets: <= 8 MB peak RAM, <= 20 ms compute per 1 s of IMU, single CPU thread, offline.
- Datasets: EuRoC MAV and TUM-VI; organizer degradation script provided.

## Challenge blueprint
Inputs and constraints: raw IMU CSV; progressive noise, bias drift, dropouts; unknown gravity alignment;
<= 8 MB peak RAM, <= 20 ms per 1 s of IMU, single CPU thread, offline; 5-second post-failure trajectory
extrapolation (dead reckoning). Tiers: T1 checkpoints, T2 blind 5 s (+20 %), T3 dropout resilience (+20 %).

Required deliverables: standalone Python script (degraded IMU CSV -> 6-DoF trajectory); trajectory file
(timestamp, x, y, z, qx, qy, qz, qw); pipeline walkthrough video (2-3 min).

Evaluation (100 points)
- Execution and efficiency, 40 %: peak memory 20 %, runtime 20 %
- Trajectory closeness, 60 %: waypoints 30 %, blind 5 s 20 %, dropout 10 %

## Input format (EuRoC IMU)
The evaluation dataset is a zipped folder of raw IMU telemetry; non-conforming submissions fail the automated
evaluation pipeline. File path imu0/data.csv, comma-separated, header
`#timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1], a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]`.
timestamp: 64-bit integer ns since Unix epoch; w: angular velocity (rad/s); a: linear acceleration (m/s^2,
includes gravity). Sample first row: `1403636579758555392,0.0124,-0.0023,0.0051,0.1241,0.0312,9.8105`.

## Output format (TUM pose)
estimated_trajectory.txt, space-separated, optional header lines starting with #, one pose per row, exactly
8 columns: timestamp (float seconds, Unix epoch), tx ty tz (m), qx qy qz qw (normalized unit quaternion, world
frame). Timestamp conversion ns -> s; exactly one pose per IMU frame (dropped or misaligned rows incur heavy
penalties or failure).

## Submission bundle and evaluation
team_name.zip: estimated_trajectory.txt, metadata.json (optional: runtime perf, compute specs), src/ (complete
source: main.py entrypoint, README.md how to run). Ranked with the open-source evo trajectory evaluation suite
against a hidden high-accuracy ground truth. Metrics: ATE (RMSE, global consistency) and RPE (local drift and
rotational error per unit of time).
