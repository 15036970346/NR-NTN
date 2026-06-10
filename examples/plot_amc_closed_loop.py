#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制 OLLA + Kalman CQI 预测闭环的关键指标 (来自 ntn-amc-closed-loop)。

输入 (默认 ntn-results/):
  amc_timeline.csv         每次新传一行: t_s,rnti,measuredCqi,predCqi,effCqi,delta,mcs,snr_db,ack
  amc_bler_convergence.csv 滑动窗口 BLER + 累积 BLER
  amc_summary.txt          (可选, 仅写日志)

输出:
  amc_closed_loop.png  四宫格:
    [1] BLER 收敛 (window + cumulative) vs target
    [2] OLLA Δ 轨迹 (per-UE)
    [3] predCqi vs measuredCqi vs actual (由 snr_db→CQI 映射) 时序
    [4] MCS 直方图 (新传计数)

用法:
  python3 contrib/geo-sat/examples/plot_amc_closed_loop.py
  python3 contrib/geo-sat/examples/plot_amc_closed_loop.py --results-dir ntn-results
"""
import argparse
import csv
import os
import sys


def map_sinr_to_cqi(snr_db):
    # 与 example 内 MapSinrToCqi 一致 (3GPP TS38.214 边界)
    bounds = [(-6.5, 1), (-4.5, 2), (-2.5, 3), (-0.5, 4),
              (1.0, 5), (3.0, 6), (5.0, 7), (7.0, 8),
              (9.0, 9), (11.0, 10), (13.0, 11), (15.0, 12),
              (17.0, 13), (19.0, 14)]
    for b, c in bounds:
        if snr_db < b:
            return c
    return 15


def read_csv_rows(path):
    rows = []
    with open(path, newline="") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    for r in reader:
        rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser(description="Plot OLLA+Kalman closed-loop AMC")
    ap.add_argument("--results-dir", default="ntn-results", help="结果目录")
    ap.add_argument("--out", default=None, help="输出 PNG (默认 <results-dir>/amc_closed_loop.png)")
    ap.add_argument("--bler-target", type=float, default=0.10, help="目标 BLER (横线)")
    args = ap.parse_args()

    tl_path = os.path.join(args.results_dir, "amc_timeline.csv")
    bc_path = os.path.join(args.results_dir, "amc_bler_convergence.csv")
    out_path = args.out or os.path.join(args.results_dir, "amc_closed_loop.png")

    if not os.path.exists(tl_path) or not os.path.exists(bc_path):
        sys.exit("找不到 CSV: 先跑 ./ns3 run ntn-amc-closed-loop")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("需要 matplotlib: pip install matplotlib")

    tl = read_csv_rows(tl_path)
    bc = read_csv_rows(bc_path)
    if not tl:
        sys.exit("timeline 为空, 仿真未产生新传")

    # ---- 数据切片 ----
    t = [float(r["t_s"]) for r in tl]
    rnti = [int(r["rnti"]) for r in tl]
    pred = [float(r["predCqi"]) for r in tl]
    meas = [float(r["measuredCqi"]) for r in tl]
    snr = [float(r["snr_db"]) for r in tl]
    actual_cqi = [map_sinr_to_cqi(s) for s in snr]   # 解码时刻真实 SINR 映射的真 CQI
    delta = [float(r["delta"]) for r in tl]
    mcs = [int(r["mcs"]) for r in tl]

    bc_t = [float(r["t_s"]) for r in bc]
    bc_win = [float(r["window_bler"]) for r in bc]
    bc_cum = [float(r["cumulative_bler"]) for r in bc]

    rnti_unique = sorted(set(rnti))

    # ---- 4 宫格 ----
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle("OLLA + Kalman closed-loop AMC (ntn-amc-closed-loop)", fontsize=13)

    # [1] BLER 收敛
    ax = axes[0, 0]
    ax.plot(bc_t, bc_cum, label="cumulative BLER", color="C0")
    ax.plot(bc_t, bc_win, label="window BLER (sliding)", color="C1", alpha=0.7)
    ax.axhline(args.bler_target, color="r", ls="--", alpha=0.6,
               label="target BLER = %.2f" % args.bler_target)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("BLER (new tx only)")
    ax.set_ylim(0, max(0.5, max(bc_win) * 1.2))
    ax.set_title("BLER convergence")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # [2] OLLA Δ 轨迹 (per-UE)
    ax = axes[0, 1]
    for r in rnti_unique:
        ts = [t[i] for i in range(len(t)) if rnti[i] == r]
        ds = [delta[i] for i in range(len(t)) if rnti[i] == r]
        ax.plot(ts, ds, label="rnti=%d" % r)
    ax.axhline(0, color="k", ls=":", alpha=0.4)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("OLLA Δ (CQI units)")
    ax.set_title("OLLA offset trajectory")
    ax.grid(True, alpha=0.3)
    if len(rnti_unique) <= 8:
        ax.legend(loc="upper right", fontsize=8)

    # [3] CQI: measured (t 时刻) vs predicted (H 步前向) vs actual (t+510 真值)
    ax = axes[1, 0]
    # 单 UE 时序; 多 UE 仅画第一个避免拥挤
    target_rnti = rnti_unique[0]
    sel = [i for i in range(len(t)) if rnti[i] == target_rnti]
    ax.plot([t[i] for i in sel], [meas[i] for i in sel], label="measured (t)", alpha=0.6, color="C2")
    ax.plot([t[i] for i in sel], [pred[i] for i in sel], label="predicted (H step ahead)",
            alpha=0.7, color="C0")
    ax.plot([t[i] for i in sel], [actual_cqi[i] for i in sel],
            label="actual @ decode (t+RTT)", alpha=0.7, color="C3", linestyle="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("CQI")
    ax.set_title("CQI: measured vs predicted vs actual at decode  (rnti=%d)" % target_rnti)
    ax.set_ylim(-0.5, 16)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    # [4] MCS 直方图
    ax = axes[1, 1]
    mcs_counts = {}
    for m in mcs:
        mcs_counts[m] = mcs_counts.get(m, 0) + 1
    xs = sorted(mcs_counts.keys())
    ys = [mcs_counts[x] for x in xs]
    ax.bar(xs, ys, color="C4", edgecolor="k", linewidth=0.5)
    ax.set_xlabel("MCS")
    ax.set_ylabel("new tx count")
    ax.set_title("MCS histogram (new tx, %d samples)" % len(mcs))
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=130)
    print("saved:", out_path)
    print("new tx total: %d   final window BLER: %.4f   cumulative BLER: %.4f"
          % (len(tl), bc_win[-1] if bc_win else 0.0, bc_cum[-1] if bc_cum else 0.0))


if __name__ == "__main__":
    main()
