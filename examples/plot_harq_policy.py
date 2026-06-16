# -*- coding: utf-8 -*-
# 绘制 4 方案 HARQ 对比图 (吞吐–时延–可靠性 三角折中)。
# 读 ntn-results/harq_policy_{a,b,c,d}.csv (各一行, 由 ntn-amc-closed-loop --harqScheme= 产出)。
# 输出 ntn-results/harq_policy_compare.png
import csv, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# CJK 字体 (与项目其他绘图脚本一致)
for p in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
          '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
          '/usr/share/fonts/truetype/arphic/uming.ttc']:
    if os.path.exists(p):
        try:
            fm.fontManager.addfont(p)
            plt.rcParams['font.sans-serif'] = [fm.FontProperties(fname=p).get_name(), 'DejaVu Sans']
            break
        except Exception:
            pass
plt.rcParams['axes.unicode_minus'] = False

RES = 'ntn-results'
SCHEMES = ['a', 'b', 'c', 'd']
LABELS = {'a': '传统\n(非预测+HARQ-N)', 'b': 'HARQ 关', 'c': '仅 AMC 预测', 'd': '本方案\n(预测式策略)'}
COLORS = {'a': '#9ca3af', 'b': '#f59e0b', 'c': '#3b82f6', 'd': '#16a34a'}

data = {}
for s in SCHEMES:
    fn = f'{RES}/harq_policy_{s}.csv'
    if not os.path.exists(fn):
        print('缺少', fn, '— 先跑 ./ns3 run "ntn-amc-closed-loop --harqScheme=%s ..."' % s)
        continue
    with open(fn) as f:
        row = list(csv.DictReader(f))[0]
    data[s] = {k: (float(v) if v.replace('.', '', 1).replace('-', '', 1).isdigit() else v)
               for k, v in row.items()}

avail = [s for s in SCHEMES if s in data]
if not avail:
    raise SystemExit('无数据: 请先生成 harq_policy_{a,b,c,d}.csv')

xs = list(range(len(avail)))
xl = [LABELS[s] for s in avail]
cols = [COLORS[s] for s in avail]


def bars(ax, key, title, better, pct=False, scale=1.0):
    vals = [data[s][key] * (100.0 if pct else 1.0) * scale for s in avail]
    b = ax.bar(xs, vals, color=cols, width=0.6)
    for r, v in zip(b, vals):
        ax.annotate(f'{v:.1f}{"%" if pct else ""}', (r.get_x() + r.get_width() / 2, v),
                    textcoords='offset points', xytext=(0, 3), ha='center', fontsize=10, fontweight='bold')
    ax.set_title(f'{title}\n({better})', fontsize=12, fontweight='bold')
    ax.set_xticks(xs); ax.set_xticklabels(xl, fontsize=9)
    ax.grid(True, axis='y', ls='--', alpha=0.4)
    ax.set_ylim(0, max(vals) * 1.20 + 1e-6)


fig, axes = plt.subplots(2, 3, figsize=(15, 8.5), dpi=140)
bars(axes[0, 0], 'residual_bler', '残余 BLER (HARQ 后)', '越低越好', pct=True)
bars(axes[0, 1], 'throughput_per_slot', '频谱效率 ΣSE/占用时隙', '越高越好')
bars(axes[0, 2], 'voice_in_budget_rate', 'VOICE 时延预算内到达率', '越高越好 ★本方案优势', pct=True)
bars(axes[1, 0], 'avg_attempts', '平均传输次数/TB', '越低越好(省重传)')
bars(axes[1, 1], 'throughput_proxy', '吞吐代理 ΣSE/s', '越高越好')

# 鲁棒性: 预测对 vs 预测错 的交付率
axr = axes[1, 2]
w = 0.38
pg = [data[s]['predgood_rate'] * 100 for s in avail]
pb = [data[s]['predbad_rate'] * 100 for s in avail]
b1 = axr.bar([x - w / 2 for x in xs], pg, width=w, color='#22c55e', label='预测对')
b2 = axr.bar([x + w / 2 for x in xs], pb, width=w, color='#ef4444', label='预测错')
axr.set_title('鲁棒性: 预测对/错 时的交付率\n(差距小=对误判鲁棒)', fontsize=12, fontweight='bold')
axr.set_xticks(xs); axr.set_xticklabels(xl, fontsize=9)
axr.set_ylim(0, 110); axr.grid(True, axis='y', ls='--', alpha=0.4); axr.legend(fontsize=9)

meta = data[avail[0]]
fig.suptitle(f'预测式 HARQ 联合策略 — 四方案对比 (numUes={int(meta["num_ues"])}, '
             f'simTime={meta["sim_time_s"]:.0f}s)  | 数据: ntn-results/harq_policy_*.csv',
             fontsize=14, fontweight='bold')
fig.tight_layout(rect=[0, 0, 1, 0.96])
out = f'{RES}/harq_policy_compare.png'
fig.savefig(out)
print('已生成', out)
