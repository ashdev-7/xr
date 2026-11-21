#!/usr/bin/env python3
"""PathGuard entrypoint (Track 03, Path Finder).

    python main.py <imu0/data.csv | dataset folder> [estimated_trajectory.txt] [--param=value ...]

The estimator itself is main.cpp (single file, no dependencies). This launcher compiles it
on first use and then replaces itself with the native binary (os.execv), so the trajectory is
computed by one single-threaded C++ process. A CPython interpreter alone needs more than the
8 MB budget, which is why the core is native code.
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "main.cpp")
BIN = os.path.join(HERE, "build", "pathguard")


def build():
    os.makedirs(os.path.dirname(BIN), exist_ok=True)
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        sys.exit("PathGuard needs a C++17 compiler (g++ or clang++) to build main.cpp")
    base = [cxx, "-O2", "-std=c++17", "-o", BIN, SRC]
    # static linking keeps peak RSS lowest; fall back to dynamic if static libs are missing
    if subprocess.run(base[:-3] + ["-static"] + base[-3:], stderr=subprocess.DEVNULL).returncode != 0:
        subprocess.run(base, check=True)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if not os.path.exists(BIN) or os.path.getmtime(BIN) < os.path.getmtime(SRC):
        build()
    os.execv(BIN, [BIN, *sys.argv[1:]])


if __name__ == "__main__":
    main()
