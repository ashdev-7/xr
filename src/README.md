# PathGuard: how to run

Track 03 (Path Finder) · Team ashdev7 · Thapar Institute of Engineering and Technology, Patiala

`main.py` is the complete, standalone estimator: Python standard library only (Python ≥ 3.8, nothing to install),
single thread, offline. It carries the five pipeline stages as embedded text and compiles them one at a time,
because parsing all the code as one block costs ~7 MB of heap; this way even memory traced from interpreter
start peaks at ~3 MB. `stages/` holds the same stage code as readable files (main.py is generated from them by
`tools/build_main.py` in the repository); they are not needed to run.

```bash
python main.py <imu0/data.csv | dataset folder | dataset .zip> [estimated_trajectory.txt] [--verbose=1]
```

- Input: EuRoC-format IMU CSV (`#timestamp [ns], w_RS_S_x..z [rad s^-1], a_RS_S_x..z [m s^-2]`). Column order,
  header presence, timestamp unit and accelerometer unit (m/s² or g) are detected automatically.
- Output: `estimated_trajectory.txt`, `timestamp tx ty tz qx qy qz qw`, exactly one pose per input row (including
  duplicated rows and the dead rows after the IMU failure), timestamps converted exactly from nanoseconds.
- `--verbose=1` prints diagnostics to stderr: detected platform, jamming level, integrity statistics.

Docker (the organizers' Python 3.11, no extra packages):

```bash
docker build -t pathguard .
docker run --rm -v "$PWD/eval:/data" pathguard /data/imu0/data.csv /data/estimated_trajectory.txt
```

Method, validation, results and limitations: see the repository README (https://github.com/ashdev-7/xr).
