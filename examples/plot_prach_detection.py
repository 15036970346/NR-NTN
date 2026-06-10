#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制 PRACH 检测 (虚警/漏检) 的 Pd/Pfa-SNR 曲线: 理论曲线 vs 实测点。

输入 (由仿真生成, 文件名前缀由 --prefix 指定):
  <prefix>_curve.txt      理论曲线  (列: snr_db,pd,pfa)
  <prefix>_empirical.txt  实测分箱  (列: snr_db,active_groups,
                          detected_groups,empirical_pd,theoretical_pd)
输出:
  <results-dir>/<prefix>.png

两个仿真的前缀:
  ntn-ra-compare   → --prefix prach_detection  (默认)
  ntn-system-sim   → --prefix sysim_prach

用法:
  python3 contrib/geo-sat/examples/plot_prach_detection.py                       # ra-compare
  python3 contrib/geo-sat/examples/plot_prach_detection.py --prefix sysim_prach  # system-sim
"""
import argparse
import csv
import os
import sys


def read_csv_rows(path):
    """读取 CSV, 跳过以 '#' 开头的注释行, 返回 dict 行列表。"""
    rows = []
    with open(path, newline="") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    for r in reader:
        rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser(description="Plot PRACH Pd/Pfa-SNR: theory vs empirical")
    ap.add_argument("--results-dir", default="ntn-results", help="结果目录")
    ap.add_argument("--prefix", default="prach_detection",
                    help="文件名前缀: prach_detection (ra-compare) | sysim_prach (system-sim)")
    ap.add_argument("--out", default=None, help="输出 PNG 路径 (默认 <results-dir>/<prefix>.png)")
    args = ap.parse_args()

    curve_path = os.path.join(args.results_dir, args.prefix + "_curve.txt")
    emp_path = os.path.join(args.results_dir, args.prefix + "_empirical.txt")
    out_path = args.out or os.path.join(args.results_dir, args.prefix + ".png")

    if not os.path.exists(curve_path):
        sys.exit("找不到理论曲线文件: %s (先跑一次对应仿真)" % curve_path)

    try:
        import matplotlib
        matplotlib.use("Agg")  # 无显示环境也能出图
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("需要 matplotlib: pip install matplotlib")

    # ---- 理论曲线 ----
    curve = read_csv_rows(curve_path)
    c_snr = [float(r["snr_db"]) for r in curve]
    c_pd = [float(r["pd"]) for r in curve]
    c_pfa = [float(r["pfa"]) for r in curve]

    # ---- 实测点 (可能不存在, 例如检测关闭时) ----
    e_snr, e_pd, e_n = [], [], []
    if os.path.exists(emp_path):
        for r in read_csv_rows(emp_path):
            e_snr.append(float(r["snr_db"]))
            e_pd.append(float(r["empirical_pd"]))
            e_n.append(int(r["active_groups"]))

    fig, ax1 = plt.subplots(figsize=(8, 5))

    # 左轴: Pd (图中文字用英文, 避免无 CJK 字体环境出现乱码)
    ax1.plot(c_snr, c_pd, "-", color="C0", label="Pd theory (curve)")
    if e_snr:
        # 散点大小随样本数变化, 提示哪些 SNR 箱统计更可信
        sizes = [20 + 4 * n ** 0.5 for n in e_n]
        ax1.scatter(e_snr, e_pd, s=sizes, color="C1", zorder=5,
                    edgecolors="k", linewidths=0.5, label="Pd empirical (binned)")
    ax1.set_xlabel("PRACH SNR (dB)")
    ax1.set_ylabel("Detection probability Pd")
    ax1.set_ylim(-0.02, 1.05)
    ax1.grid(True, alpha=0.3)

    # 右轴: Pfa
    ax2 = ax1.twinx()
    ax2.plot(c_snr, c_pfa, "--", color="C3", alpha=0.7, label="Pfa theory")
    ax2.set_ylabel("False-alarm probability Pfa")
    pfa_max = max(c_pfa) if max(c_pfa) > 0 else 1e-3
    ax2.set_ylim(0, pfa_max * 3)

    # 合并图例
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="center right")

    src = "ntn-system-sim" if args.prefix.startswith("sysim") else "ntn-ra-compare"
    plt.title("PRACH Pd/Pfa vs SNR: theory vs empirical (%s)" % src)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print("已保存: %s" % out_path)
    if e_snr:
        print("实测 SNR 箱: %d 个, 样本范围 %d~%d 组/箱"
              % (len(e_snr), min(e_n), max(e_n)))


if __name__ == "__main__":
    main()
