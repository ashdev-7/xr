# PathGuard (Track 03, Path Finder) - reproducible build and run
#
#   docker build -t pathguard .
#   docker run --rm -v "$PWD/data:/data" pathguard /data/imu0/data.csv /data/estimated_trajectory.txt
#
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends g++ python3 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY main.cpp main.py Makefile ./
RUN make
ENTRYPOINT ["/app/build/pathguard"]
