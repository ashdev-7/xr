"""Ground-truth labels for the official degradation (development/evaluation only, never used by main.py).

Re-runs the exact RNG sequence of tools/degrade_imu_official.py:degrade() and returns, for every
OUTPUT row, what was done to it: spike, stuck (frozen copy), duplicate, failed (zeroed), plus the
input-row index. verify() asserts that our reproduction is bit-identical to the official output.
"""
import os, sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import degrade_imu_official as O  # noqa: E402


def degrade_with_labels(df, cfg, seed):
    rng = np.random.default_rng(seed)
    cols = list(df.columns)
    ts_col, acc_cols, gyr_cols = cols[0], cols[1:4], cols[4:7]
    ts = df[ts_col].to_numpy(dtype=float)
    acc = df[acc_cols].to_numpy(dtype=float)
    gyr = df[gyr_cols].to_numpy(dtype=float)
    n = len(df)
    ts_scale, _ = O.detect_ts_scale(ts, "auto")
    acc_scale, _ = O.detect_accel_scale(acc, "auto")
    gyr_scale = 1.0
    t_sec = (ts - ts[0]) * ts_scale
    T = t_sec[-1] if t_sec[-1] > 0 else 1.0
    dt = np.diff(t_sec, prepend=np.nan); dt[0] = np.nanmedian(dt[1:]); dt = np.clip(dt, 1e-4, None)
    sqrt_dt = np.sqrt(dt)[:, None]
    sev = O.severity(t_sec / T, cfg); sev_c = sev[:, None]
    if T > cfg.get("failure_seconds", 0.0) + cfg["protect_seconds"]:
        failed = t_sec >= (T - cfg["failure_seconds"])
    else:
        failed = np.zeros(n, dtype=bool)
    active = ~(t_sec < cfg["protect_seconds"]) & ~failed
    acc_n = acc + rng.standard_normal((n, 3)) * cfg["accel_white_max"] * acc_scale * sev_c
    gyr_n = gyr + rng.standard_normal((n, 3)) * cfg["gyro_white_max"] * gyr_scale * sev_c
    acc_bias = np.cumsum(rng.standard_normal((n, 3)) * cfg["accel_bias_rw_max"] * acc_scale * sqrt_dt * sev_c, axis=0)
    gyr_bias = np.cumsum(rng.standard_normal((n, 3)) * cfg["gyro_bias_rw_max"] * gyr_scale * sqrt_dt * sev_c, axis=0)
    acc_sf = np.cumsum(rng.standard_normal((n, 3)) * cfg["scale_rw_max"] * sqrt_dt * sev_c, axis=0)
    gyr_sf = np.cumsum(rng.standard_normal((n, 3)) * cfg["scale_rw_max"] * sqrt_dt * sev_c, axis=0)
    acc_n = acc_n * (1.0 + acc_sf) + acc_bias
    gyr_n = gyr_n * (1.0 + gyr_sf) + gyr_bias
    spike = (rng.random(n) < cfg["spike_prob_max"] * sev) & active
    k = int(spike.sum())
    if k:
        sgn = rng.choice([-1.0, 1.0], size=(k, 3))
        mag = rng.uniform(0.5, 1.5, size=(k, 3))
        acc_n[spike] += sgn * mag * cfg["accel_spike_mag"] * acc_scale * sev[spike, None]
        gyr_n[spike] += sgn * mag * cfg["gyro_spike_mag"] * gyr_scale * sev[spike, None]
    stuck_start = (rng.random(n) < cfg["stuck_prob_max"] * sev) & active
    stuck_idx = np.flatnonzero(stuck_start)
    stuck_lens = O.run_lengths(rng, len(stuck_idx), cfg["stuck_len"], sev[stuck_idx])
    stuck = np.zeros(n, bool)
    for i, L in zip(stuck_idx, stuck_lens):
        j = min(n, i + 1 + L)
        acc_n[i + 1:j] = acc_n[i]; gyr_n[i + 1:j] = gyr_n[i]; stuck[i + 1:j] = True
    keep = np.ones(n, dtype=bool)
    single_drop = (rng.random(n) < cfg["drop_prob_max"] * sev) & active
    keep &= ~single_drop
    burst_start = (rng.random(n) < cfg["burst_prob_max"] * sev) & active
    burst_idx = np.flatnonzero(burst_start)
    burst_lens = O.run_lengths(rng, len(burst_idx), cfg["burst_len"], sev[burst_idx])
    for i, L in zip(burst_idx, burst_lens):
        keep[i:min(n, i + L)] = False
    keep[~active] = True
    keep[0] = keep[-1] = True
    if cfg.get("failure_seconds", 0.0) > 0:
        acc_n[failed] = 0.0; gyr_n[failed] = 0.0
    out = pd.DataFrame({ts_col: df[ts_col].to_numpy()})
    for c, j in zip(acc_cols, range(3)):
        out[c] = acc_n[:, j]
    for c, j in zip(gyr_cols, range(3)):
        out[c] = gyr_n[:, j]
    src = np.arange(n)[keep]
    out = out[keep].reset_index(drop=True)
    sev_kept = sev[keep]
    dup = rng.random(len(out)) < cfg["dup_prob_max"] * sev_kept
    counts = 1 + dup.astype(int)
    rep = np.repeat(np.arange(len(out)), counts)
    out = out.iloc[rep].reset_index(drop=True)
    out[acc_cols + gyr_cols] = out[acc_cols + gyr_cols].round(6)
    src = src[rep]
    is_dup = np.zeros(len(rep), bool); is_dup[1:] = rep[1:] == rep[:-1]
    labels = pd.DataFrame(dict(src=src, spike=spike[src], stuck=stuck[src], dup=is_dup, failed=failed[src],
                               sev=sev[src]))
    # truth (clean) values of the kept rows, for error analysis
    clean = df.iloc[src].reset_index(drop=True)
    return out, labels, clean


def verify(df, seed=42):
    cfg = dict(O.CONFIG)
    a, _ = O.degrade(df, cfg, seed)
    b, lab, _ = degrade_with_labels(df, cfg, seed)
    assert a.shape == b.shape and (a.to_numpy() == b.to_numpy()).all(), "label reproduction differs from official output"
    return lab
