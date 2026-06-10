#!/usr/bin/env python3
"""画 ns-3 3GPP HTTP 模型三类分布族的拟合曲线。

经验值来自 scratch/http-dist-sample.cc 导出的样本(真实 RNG, 含截断),
理论曲线来自解析公式。截断由 Python 在观测支撑集上数值归一化处理。

布局: 5 个变量(行) × {PDF/PMF, CDF}(列) = 5×2。

用法:
    python3 plot_http_dist.py http-dist-samples.txt -o http-dist.png
"""
import argparse
import re

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats


def parse_header(path):
    """从文件头注释里读出每个分布的理论参数。"""
    params = {}
    with open(path) as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            kvs = dict(re.findall(r"(\w+)=([\-\d.eE]+)", line))
            if "main_object_size" in line:
                params["main"] = dict(
                    mu=float(kvs["mu"]),
                    sigma=float(kvs["sigma"]),
                    lo=float(kvs["min"]),
                    hi=float(kvs["max"]),
                )
            elif "embedded_object_size" in line:
                params["embedded"] = dict(
                    mu=float(kvs["mu"]),
                    sigma=float(kvs["sigma"]),
                    lo=float(kvs["min"]),
                    hi=float(kvs["max"]),
                )
            elif "num_embedded" in line:
                params["emb"] = dict(mean=float(kvs["mean"]), bound=int(float(kvs["bound"])))
            elif "reading_time" in line:
                params["read"] = dict(mean=float(kvs["mean"]))
            elif "parsing_time" in line:
                params["parse"] = dict(mean=float(kvs["mean"]))
    return params


def read_samples(path):
    comment_lines = 0
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                comment_lines += 1
            else:
                break
    return np.genfromtxt(path, skip_header=comment_lines, names=True, dtype=float, encoding="utf-8")


def plot_continuous(
    ax_pdf,
    ax_cdf,
    data,
    theory_pdf,
    theory_cdf,
    title,
    xlabel,
    trunc_lo,
    trunc_hi,
    logx=False,
):
    """连续分布: 直方图(密度) + 截断理论PDF; 经验CDF + 截断理论CDF。"""
    lo = max(float(np.min(data)), trunc_lo)
    hi = min(float(np.max(data)), trunc_hi)
    if logx:
        grid = np.logspace(np.log10(max(lo, 1)), np.log10(hi), 400)
    else:
        grid = np.linspace(lo, hi, 400)

    trunc_norm = theory_cdf(trunc_hi) - theory_cdf(trunc_lo)
    if trunc_norm <= 0:
        raise ValueError(f"invalid truncation range for {title}")

    pdf = theory_pdf(grid)
    pdf = pdf / trunc_norm

    hist_bins = np.logspace(np.log10(max(lo, 1)), np.log10(hi), 80) if logx else 80
    ax_pdf.hist(data, bins=hist_bins, density=True, alpha=0.5, color="C0", label="经验(采样)")
    ax_pdf.plot(grid, pdf, "C3-", lw=2, label="理论PDF(真实截断)")
    ax_pdf.set_title(f"{title} — PDF")
    ax_pdf.set_xlabel(xlabel)
    ax_pdf.set_ylabel("密度")
    ax_pdf.legend()

    # 经验CDF
    xs = np.sort(data)
    ecdf = np.arange(1, len(xs) + 1) / len(xs)
    # 理论CDF, 按真实截断区间 [trunc_lo, trunc_hi] 重标定
    Flo, Fhi = theory_cdf(trunc_lo), theory_cdf(trunc_hi)
    tcdf = (theory_cdf(grid) - Flo) / (Fhi - Flo)

    ax_cdf.plot(xs, ecdf, "C0-", lw=1.5, label="经验CDF")
    ax_cdf.plot(grid, tcdf, "C3--", lw=2, label="理论CDF")
    ks = stats.kstest(
        data,
        lambda x: np.clip((theory_cdf(x) - Flo) / (Fhi - Flo), 0.0, 1.0),
    )
    ax_cdf.set_title(f"{title} — CDF (KS D={ks.statistic:.3f})")
    ax_cdf.set_xlabel(xlabel)
    ax_cdf.set_ylabel("累积概率")
    ax_cdf.legend()
    if logx:
        ax_pdf.set_xscale("log")
        ax_cdf.set_xscale("log")


def plot_discrete(ax_pmf, ax_cdf, data, mean, bound, title):
    """离散分布(内嵌对象数): 经验频率柱 + 截断Poisson PMF; 阶梯CDF。"""
    ks_vals = np.arange(0, bound)
    pmf = stats.poisson.pmf(ks_vals, mean)
    pmf = pmf / pmf.sum()  # 截断到 [0,bound) 后归一

    counts = np.bincount(data, minlength=bound)[:bound]
    emp = counts / counts.sum()

    ax_pmf.bar(ks_vals - 0.18, emp, width=0.36, color="C0", label="经验频率")
    ax_pmf.bar(ks_vals + 0.18, pmf, width=0.36, color="C3", alpha=0.8, label="理论PMF(截断Poisson)")
    ax_pmf.set_title(f"{title} — PMF")
    ax_pmf.set_xlabel("内嵌对象数")
    ax_pmf.set_ylabel("概率")
    ax_pmf.legend()

    ax_cdf.step(ks_vals, np.cumsum(emp), where="post", color="C0", lw=1.5, label="经验CDF")
    ax_cdf.step(ks_vals, np.cumsum(pmf), where="post", color="C3", ls="--", lw=2, label="理论CDF")
    ax_cdf.set_title(f"{title} — CDF")
    ax_cdf.set_xlabel("内嵌对象数")
    ax_cdf.set_ylabel("累积概率")
    ax_cdf.legend()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("samples", help="http-dist-sample.cc 导出的 txt")
    ap.add_argument("-o", "--out", default="http-dist.png")
    args = ap.parse_args()

    plt.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC", "Noto Sans CJK JP",
        "WenQuanYi Zen Hei", "SimHei", "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False

    p = parse_header(args.samples)
    data = read_samples(args.samples)
    main_sz = data["main_object_size"]
    embedded_sz = data["embedded_object_size"]
    num_emb = data["num_embedded"].astype(int)
    read_t = data["reading_time"]
    parse_t = data["parsing_time"]

    fig, axes = plt.subplots(5, 2, figsize=(13, 21))

    # 1) 主对象大小: Truncated LogNormal
    mu, sigma = p["main"]["mu"], p["main"]["sigma"]
    plot_continuous(
        axes[0, 0], axes[0, 1], main_sz,
        theory_pdf=lambda x: stats.lognorm.pdf(x, s=sigma, scale=np.exp(mu)),
        theory_cdf=lambda x: stats.lognorm.cdf(x, s=sigma, scale=np.exp(mu)),
        title="主对象大小 Truncated LogNormal",
        xlabel="字节",
        trunc_lo=p["main"]["lo"],
        trunc_hi=p["main"]["hi"],
        logx=True,
    )

    # 2) 内嵌对象大小: Truncated LogNormal
    mu, sigma = p["embedded"]["mu"], p["embedded"]["sigma"]
    plot_continuous(
        axes[1, 0], axes[1, 1], embedded_sz,
        theory_pdf=lambda x: stats.lognorm.pdf(x, s=sigma, scale=np.exp(mu)),
        theory_cdf=lambda x: stats.lognorm.cdf(x, s=sigma, scale=np.exp(mu)),
        title="内嵌对象大小 Truncated LogNormal",
        xlabel="字节",
        trunc_lo=p["embedded"]["lo"],
        trunc_hi=p["embedded"]["hi"],
        logx=True,
    )

    # 3) 内嵌对象数: 截断 Poisson
    plot_discrete(axes[2, 0], axes[2, 1], num_emb,
                  mean=p["emb"]["mean"], bound=p["emb"]["bound"],
                  title="内嵌对象数 Poisson")

    # 4) 阅读时间: Exponential
    rmean = p["read"]["mean"]
    plot_continuous(
        axes[3, 0], axes[3, 1], read_t,
        theory_pdf=lambda x: stats.expon.pdf(x, scale=rmean),
        theory_cdf=lambda x: stats.expon.cdf(x, scale=rmean),
        title="阅读时间 Exponential",
        xlabel="秒",
        trunc_lo=0.0,
        trunc_hi=float(np.max(read_t)),
        logx=False,
    )

    # 5) 解析时间: Exponential
    pmean = p["parse"]["mean"]
    plot_continuous(
        axes[4, 0], axes[4, 1], parse_t,
        theory_pdf=lambda x: stats.expon.pdf(x, scale=pmean),
        theory_cdf=lambda x: stats.expon.cdf(x, scale=pmean),
        title="解析时间 Exponential",
        xlabel="秒",
        trunc_lo=0.0,
        trunc_hi=float(np.max(parse_t)),
        logx=False,
    )

    sample_name = args.samples.rsplit("/", 1)[-1]
    fig.suptitle(f"HTTP Distribution Fit — {sample_name}", fontsize=16, y=0.995)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
