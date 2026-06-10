#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制 ntn-system-sim 的 Msg4 PDCCH/PDSCH BLER-SNR 曲线: 理论 + 实测 Msg4 成功率。

输入 (ntn-results/):
  msg4_pdcch_bler_curve.txt    snr_db,bler             (PDCCH 理论)
  msg4_pdsch_bler_curve.txt    snr_db,bler             (PDSCH 理论)
  msg4_demod_empirical.txt     snr_db,attempts,success,emp_msg4_success,
                               theo_pdcch_bler,theo_pdsch_bler,theo_combined_success

输出:
  ntn-results/msg4_demod.png   三层叠加: PDCCH BLER 曲线 + PDSCH BLER 曲线 +
                               合并 Msg4 成功率 (理论线 + 实测散点, 散点大小∝样本)

用法:
  python3 contrib/geo-sat/examples/plot_msg4_demod.py
  python3 contrib/geo-sat/examples/plot_msg4_demod.py --results-dir ntn-results
"""
import argparse
import csv
import os
import sys


def read_csv_rows(path):
    rows = []
    with open(path, newline="") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    for r in reader:
        rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser(description="Plot Msg4 PDCCH/PDSCH BLER + combined success")
    ap.add_argument("--results-dir", default="ntn-results", help="结果目录")
    ap.add_argument("--out", default=None, help="输出 PNG (默认 <dir>/msg4_demod.png)")
    args = ap.parse_args()

    pdcch_path = os.path.join(args.results_dir, "msg4_pdcch_bler_curve.txt")
    pdsch_path = os.path.join(args.results_dir, "msg4_pdsch_bler_curve.txt")
    emp_path = os.path.join(args.results_dir, "msg4_demod_empirical.txt")
    out_path = args.out or os.path.join(args.results_dir, "msg4_demod.png")

    missing = [p for p in (pdcch_path, pdsch_path) if not os.path.exists(p)]
    if missing:
        sys.exit("找不到 BLER 曲线文件: %s (先跑 ./ns3 run ntn-system-sim)" % ", ".join(missing))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("需要 matplotlib: pip install matplotlib")

    pdcch = read_csv_rows(pdcch_path)
    pdsch = read_csv_rows(pdsch_path)
    p_snr = [float(r["snr_db"]) for r in pdcch]
    p_bler = [float(r["bler"]) for r in pdcch]
    s_snr = [float(r["snr_db"]) for r in pdsch]
    s_bler = [float(r["bler"]) for r in pdsch]

    # 合并理论 Msg4 成功率: (1-PDCCH_BLER) * (1-PDSCH_BLER)
    # 用 snr 对齐: 两条曲线 snr 是相同采样
    if p_snr == s_snr:
        snr_combo = p_snr
        succ_combo = [(1 - p_bler[i]) * (1 - s_bler[i]) for i in range(len(snr_combo))]
    else:
        # 若不一致则各自画, 不合成 (避免插值复杂度)
        snr_combo = []
        succ_combo = []

    # 实测点 (可能没有, 例如 Msg4 BLER 关时)
    e_snr, e_emp_succ, e_att, e_theo = [], [], [], []
    if os.path.exists(emp_path):
        for r in read_csv_rows(emp_path):
            e_snr.append(float(r["snr_db"]))
            e_emp_succ.append(float(r["emp_msg4_success"]))
            e_att.append(int(r["attempts"]))
            e_theo.append(float(r["theo_combined_success"]))

    fig, ax1 = plt.subplots(figsize=(9, 5.5))

    # 左轴: BLER 曲线 (PDCCH + PDSCH)
    ax1.plot(p_snr, p_bler, "-", color="C0", label="PDCCH BLER (theory)")
    ax1.plot(s_snr, s_bler, "-", color="C3", label="PDSCH BLER (theory)")
    ax1.set_xlabel("PRACH SNR (dB)")
    ax1.set_ylabel("BLER (PDCCH / PDSCH)")
    ax1.set_ylim(-0.02, 1.05)
    ax1.grid(True, alpha=0.3)

    # 右轴: 合并 Msg4 成功率 (理论曲线 + 实测散点)
    ax2 = ax1.twinx()
    if snr_combo:
        ax2.plot(snr_combo, succ_combo, "--", color="C2", lw=1.8,
                 label="Msg4 success (theory: (1-Bp)·(1-Bs))")
    if e_snr:
        sizes = [20 + 4 * (n ** 0.5) for n in e_att]
        ax2.scatter(e_snr, e_emp_succ, s=sizes, color="C2", zorder=5,
                    edgecolors="k", linewidths=0.5, label="Msg4 success (empirical)")
    ax2.set_ylabel("Msg4 combined success rate")
    ax2.set_ylim(-0.02, 1.05)

    # 合并图例
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="center right", fontsize=9)

    plt.title("Msg4 PDCCH/PDSCH BLER + combined success (ntn-system-sim)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print("saved:", out_path)
    if e_snr:
        print("empirical bins: %d, samples %d~%d / bin"
              % (len(e_snr), min(e_att), max(e_att)))


if __name__ == "__main__":
    main()
