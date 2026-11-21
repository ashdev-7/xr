# PathGuard (Track 03, Path Finder): reproducible run.
# The estimator is a single pure-Python standard-library script: no pip packages, no compiler.
#
#   docker build -t pathguard src/
#   docker run --rm -v "$PWD/eval:/data" pathguard /data/imu0/data.csv /data/estimated_trajectory.txt
#
FROM python:3.11-slim
WORKDIR /app
COPY *.py ./
ENTRYPOINT ["python", "-I", "/app/main.py"]
