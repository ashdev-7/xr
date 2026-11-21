# PathGuard: how to run

Track 03 (Path Finder) · Team ashdev7 · Thapar Institute of Engineering and Technology, Patiala

`main.py` is the entry point; the pipeline is in five stage files next to it (`pg_ingest.py`, `pg_math.py`,
`pg_attitude.py`, `pg_translation.py`, `pg_output.py`). Python standard library only (Python ≥ 3.8, nothing to
install), single thread, offline. Keep the six `.py` files together. `main.py` compiles the stages one at a time
because parsing all the code as one file costs ~7 MB of heap; this way the worst-case traced peak is ~3 MB.
If the stage files are missing, `main.py` still writes a well-formed stand-still trajectory and says so on stderr.

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
