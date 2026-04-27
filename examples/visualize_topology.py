#!/usr/bin/env python3
"""
NR NTN GEO卫星拓扑可视化
生成PNG图片展示拓扑结构
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch, ConnectionStyle
import numpy as np

def generate_topology_image():
    # 创建图形
    fig, ax = plt.subplots(1, 1, figsize=(16, 12))
    
    # 设置背景
    ax.set_facecolor('#1a1a2e')
    
    # 颜色定义
    colors = {
        'satellite': '#FFD700',    # 金色
        'gateway': '#4169E1',      # 皇家蓝
        'beams': ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500'],
        'ue': '#FFFFFF'
    }
    
    # 节点位置 (画布中心为500,500)
    center_x, center_y = 500, 500
    
    # ===== 绘制GEO卫星 =====
    sat_x, sat_y = 500, 650
    sat_circle = Circle((sat_x, sat_y), 25, color=colors['satellite'], ec='white', linewidth=2)
    ax.add_patch(sat_circle)
    ax.text(sat_x, sat_y, 'GEO\n35786km', ha='center', va='center', fontsize=8, fontweight='bold', color='black')
    
    # 绘制卫星到信关站的链路
    ax.annotate('', xy=(center_x, center_y+50), xytext=(sat_x, sat_y-25),
                arrowprops=dict(arrowstyle='->', color='yellow', lw=2))
    ax.text((sat_x+center_x)/2, (sat_y+center_y+50)/2, 'Feeder Link\n(~600ms RTT)', 
            ha='center', va='bottom', fontsize=8, color='yellow')
    
    # ===== 绘制信关站 =====
    gw_x, gw_y = center_x, center_y + 50
    gw_rect = FancyBboxPatch((gw_x-30, gw_y-20), 60, 40, boxstyle="round,pad=0.05", 
                             facecolor=colors['gateway'], ec='white', linewidth=2)
    ax.add_patch(gw_rect)
    ax.text(gw_x, gw_y, 'Gateway', ha='center', va='center', fontsize=9, fontweight='bold', color='white')
    
    # ===== 绘制7个波束 (蜂窝状) =====
    beam_positions = [
        (center_x, center_y),      # Beam 0 - 中心
        (center_x + 120, center_y),  # Beam 1 - 右
        (center_x + 60, center_y + 104),   # Beam 2 - 右上
        (center_x - 60, center_y + 104),   # Beam 3 - 左上
        (center_x - 120, center_y),  # Beam 4 - 左
        (center_x - 60, center_y - 104),   # Beam 5 - 左下
        (center_x + 60, center_y - 104),   # Beam 6 - 右下
    ]
    
    beam_radius = 80  # 波束覆盖半径
    
    # 绘制波束覆盖圆
    for i, (bx, by) in enumerate(beam_positions):
        # 波束覆盖区域 (半透明)
        beam_circle = Circle((bx, by), beam_radius, color=colors['beams'][i], alpha=0.15, ec=colors['beams'][i], linewidth=2, linestyle='--')
        ax.add_patch(beam_circle)
        
        # 波束节点
        beam_node = Circle((bx, by), 12, color=colors['beams'][i], ec='white', linewidth=1.5)
        ax.add_patch(beam_node)
        ax.text(bx, by, f'B{i}', ha='center', va='center', fontsize=8, fontweight='bold', color='black')
        
        # 信关站到波束的链路
        ax.annotate('', xy=(bx, by), xytext=(gw_x, gw_y-20),
                    arrowprops=dict(arrowstyle='->', color=colors['beams'][i], lw=1.5, alpha=0.7))
    
    # ===== 生成UE位置 =====
    np.random.seed(42)  # 固定随机种子
    ues = []
    for i, (bx, by) in enumerate(beam_positions):
        for j in range(3):  # 每个波束3个UE
            angle = np.random.uniform(0, 2*np.pi)
            distance = np.random.uniform(20, beam_radius * 0.7)
            ue_x = bx + distance * np.cos(angle)
            ue_y = by + distance * np.sin(angle)
            ues.append({
                'x': ue_x, 
                'y': ue_y, 
                'beam': i,
                'id': i*3 + j
            })
    
    # 绘制UE
    for ue in ues:
        ue_circle = Circle((ue['x'], ue['y']), 6, color='white', alpha=0.8, ec=colors['beams'][ue['beam']], linewidth=1)
        ax.add_patch(ue_circle)
        ax.text(ue['x'], ue['y'], f"UE{ue['id']}", ha='center', va='center', fontsize=6, color='black')
        
        # UE到波束的链路
        ax.plot([ue['x'], beam_positions[ue['beam']][0]], 
                [ue['y'], beam_positions[ue['beam']][1]], 
                color=colors['beams'][ue['beam']], alpha=0.3, linewidth=0.5)
    
    # ===== 图例 =====
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=colors['satellite'], markersize=15, label='GEO Satellite'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=colors['gateway'], markersize=15, label='Gateway'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=colors['beams'][0], markersize=12, label='Beam (7-color reuse)'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', markersize=8, label='User Terminal (UE)'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=10, facecolor='#2d2d44', edgecolor='white', labelcolor='white')
    
    # ===== 信息框 =====
    info_text = """NR NTN GEO Satellite System
    ========================
    Frequency Band: S-band (2 GHz)
    Nominal Total Beam Bandwidth: 35 MHz
    Per Beam Bandwidth: 5 MHz
    Nominal Total PRB Baseline: 175 PRB (7 x 25 PRB)
    SCS: 15 kHz

    Frequency Reuse: 7-color
    Beam Carrier Layout: 7 x 5 MHz (25 PRB/carrier)
    
    HARQ: IR (max 4 retx)
    RTT Delay: ~600 ms"""
    
    props = dict(boxstyle='round', facecolor='#2d2d44', alpha=0.9, edgecolor='white')
    ax.text(0.02, 0.02, info_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='bottom', horizontalalignment='left',
            bbox=props, color='white', family='monospace')
    
    # ===== 标题 =====
    ax.set_title('NR NTN GEO Satellite Topology\n7-Beam Coverage with 7-Color Frequency Reuse', 
                 fontsize=14, fontweight='bold', color='white', pad=20)
    
    # ===== 坐标轴设置 =====
    ax.set_xlim(200, 800)
    ax.set_ylim(250, 750)
    ax.set_aspect('equal')
    ax.axis('off')
    
    plt.tight_layout()
    
    # 保存图片
    output_path = '/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/examples/topology.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e', edgecolor='none')
    print(f"Topology image saved to: {output_path}")
    
    # 同时保存PDF矢量格式
    pdf_path = '/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/examples/topology.pdf'
    plt.savefig(pdf_path, bbox_inches='tight', facecolor='#1a1a2e', edgecolor='none')
    print(f"Topology PDF saved to: {pdf_path}")
    
    plt.close()

if __name__ == "__main__":
    print("Generating NR NTN GEO Satellite Topology Visualization...")
    generate_topology_image()
    print("Done!")
