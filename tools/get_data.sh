#!/usr/bin/env bash
# Fetch EuRoC ground truth (from the OpenVINS repository) and build degraded practice data.
set -euo pipefail
tmp=$(mktemp -d)
git clone -q --filter=blob:none --no-checkout --depth 1 https://github.com/rpng/open_vins "$tmp/ov"
git -C "$tmp/ov" sparse-checkout set ov_data/euroc_mav
git -C "$tmp/ov" checkout -q HEAD
mkdir -p data/gt
cp "$tmp"/ov/ov_data/euroc_mav/*.csv data/gt/
# V1_01_easy.csv in that folder is a 20 Hz copy; the 200 Hz file is V1_01_easy_original.csv
mv data/gt/V1_01_easy.csv data/gt/V1_01_easy_20hz.bak
mv data/gt/V1_01_easy_original.csv data/gt/V1_01_easy.csv
rm -rf "$tmp"
python3 tools/make_dev_data.py
