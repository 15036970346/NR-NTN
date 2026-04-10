#!/usr/bin/env python3
"""
拓扑可视化生成脚本 - NR NTN GEO卫星仿真
功能：生成NetAnim可视化文件，展示：
- GEO卫星位置
- 7个波束覆盖区域
- 多波束频率复用(7-color)
- 用户终端分布
- 信关站

使用方法: python3 generate_topology.py
输出: topology.xml (NetAnim可读)
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import math
import os

class TopologyGenerator:
    def __init__(self):
        # 创建NetAnim XML结构
        self.xml = ET.Element("NetAnim", {"version": "3.0"})
        
        # 拓扑参数
        self.satellite_height = 35786000  # GEO卫星高度 (m)
        self.earth_radius = 6371000       # 地球半径 (m)
        self.satellite_lon = 110.5        # 卫星经度 (度)
        self.beam_radius = 500000         # 单波束覆盖半径 (m)
        self.num_beams = 7               # 波束数量
        self.ue_positions = []           # UE位置列表
        
        # 颜色定义 (7色复用)
        self.beam_colors = [
            "#FF0000",  # 红色 - Beam 0
            "#00FF00",  # 绿色 - Beam 1
            "#0000FF",  # 蓝色 - Beam 2
            "#FFFF00",  # 黄色 - Beam 3
            "#FF00FF",  # 紫色 - Beam 4
            "#00FFFF",  # 青色 - Beam 5
            "#FFA500",  # 橙色 - Beam 6
        ]
        
    def generate_satellite_position(self):
        """生成GEO卫星位置"""
        # GEO卫星在36000km高度
        # 这里简化为2D坐标
        return (500, 500)  # 画布中心
    
    def generate_beam_positions(self):
        """生成7个波束的中心位置 (蜂窝状布局)"""
        beam_positions = []
        center_x, center_y = self.generate_satellite_position()
        
        # 7个波束的蜂窝状分布
        # 中心1个 + 周围6个
        angles = [0, 60, 120, 180, 240, 300]  # 周围6个的角度
        
        # 中心波束
        beam_positions.append((center_x, center_y, 0))
        
        # 周围6个波束 (六边形布局)
        radius = 200  # 波束间距
        for i, angle in enumerate(angles):
            angle_rad = math.radians(angle)
            x = center_x + radius * math.cos(angle_rad)
            y = center_y + radius * math.sin(angle_rad)
            beam_positions.append((x, y, i + 1))
            
        return beam_positions
    
    def generate_ue_positions(self, ues_per_beam=3):
        """生成用户终端位置 (每个波束若干UE)"""
        beam_positions = self.generate_beam_positions()
        ue_positions = []
        ue_id = 0
        
        for beam_x, beam_y, beam_id in beam_positions:
            for i in range(ues_per_beam):
                # 在波束覆盖范围内随机分布
                import random
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(0, self.beam_radius * 0.8)
                
                x = beam_x + distance * math.cos(angle)
                y = beam_y + distance * math.sin(angle)
                
                ue_positions.append({
                    'id': ue_id,
                    'x': x,
                    'y': y,
                    'beam_id': beam_id,
                    'color': self.beam_colors[beam_id % 7]
                })
                ue_id += 1
                
        self.ue_positions = ue_positions
        return ue_positions
    
    def create_position_element(self, node_id, x, y, color="#000000"):
        """创建节点位置元素"""
        pos = ET.SubElement(self.xml, "Position", {
            "x": str(x),
            "y": str(y),
            "z": "0"
        })
        return pos
    
    def create_node_element(self, node_id, name, node_type, x, y, color="#000000"):
        """创建完整节点元素"""
        # 节点描述
        desc = ET.SubElement(self.xml, "Node", {
            "id": str(node_id),
            "name": name,
            "type": node_type,
            "color": color
        })
        
        # 位置
        pos = ET.SubElement(desc, "Position", {
            "x": str(x),
            "y": str(y),
            "z": "0"
        })
        
        return desc
    
    def create_link_element(self, src_id, dst_id, link_type="wireless"):
        """创建链路元素"""
        link = ET.SubElement(self.xml, "Link", {
            "src": str(src_id),
            "dst": str(dst_id),
            "type": link_type
        })
        return link
    
    def add_satellite(self, node_id=0):
        """添加GEO卫星节点"""
        x, y = self.generate_satellite_position()
        return self.create_node_element(
            node_id, 
            f"GEO Satellite (H={self.satellite_height/1000:.0f}km)", 
            "satellite",
            x, y, 
            "#FFD700"  # 金色
        )
    
    def add_gateway(self, node_id=1, x=300, y=500):
        """添加信关站节点"""
        return self.create_node_element(
            node_id,
            "Gateway Station",
            "gateway",
            x, y,
            "#4169E1"  # 皇家蓝
        )
    
    def add_beams(self, start_node_id=2):
        """添加7个波束覆盖区域"""
        beam_positions = self.generate_beam_positions()
        beam_nodes = []
        
        for i, (x, y, beam_id) in enumerate(beam_positions):
            node = self.create_node_element(
                start_node_id + i,
                f"Beam {beam_id} (Color {i})",
                "beam",
                x, y,
                self.beam_colors[i]
            )
            beam_nodes.append((start_node_id + i, beam_id))
            
            # 添加波束覆盖圆 (用注释表示)
            print(f"Beam {beam_id}: center=({x:.0f},{y:.0f}), radius={self.beam_radius}")
            
        return beam_nodes
    
    def add_ues(self, start_node_id=10):
        """添加用户终端"""
        if not self.ue_positions:
            self.generate_ue_positions()
            
        ue_nodes = []
        for ue in self.ue_positions:
            node = self.create_node_element(
                start_node_id + ue['id'],
                f"UE-{ue['id']} (Beam-{ue['beam_id']})",
                "ue",
                ue['x'],
                ue['y'],
                ue['color']
            )
            ue_nodes.append(start_node_id + ue['id'])
            
        return ue_nodes
    
    def add_links(self, satellite_id, gateway_id, beam_ids, ue_ids):
        """添加链路连接"""
        links = []
        
        # 卫星到信关站 (馈电链路)
        links.append(self.create_link_element(satellite_id, gateway_id, "feeder"))
        
        # 信关站到每个波束 (馈电链路)
        for beam_id in beam_ids:
            links.append(self.create_link_element(gateway_id, beam_id, "feeder"))
            
        # 波束到UE (用户链路)
        ue_per_beam = len(ue_ids) // len(beam_ids)
        for i, beam_id in enumerate(beam_ids):
            start_ue = i * ue_per_beam
            end_ue = start_ue + ue_per_beam
            for ue_id in ue_ids[start_ue:end_ue]:
                links.append(self.create_link_element(beam_id, ue_id, "user"))
                
        return links
    
    def generate_topology_xml(self):
        """生成完整拓扑XML"""
        # 生成UE位置
        self.generate_ue_positions(ues_per_beam=3)
        
        # 添加所有节点
        satellite_id = 0
        gateway_id = 1
        beam_start_id = 2
        ue_start_id = 10
        
        self.add_satellite(satellite_id)
        self.add_gateway(gateway_id)
        beam_nodes = self.add_beams(beam_start_id)
        ue_nodes = self.add_ues(ue_start_id)
        
        # 添加链路
        beam_ids = [node_id for node_id, _ in beam_nodes]
        self.add_links(satellite_id, gateway_id, beam_ids, ue_nodes)
        
        return self.xml
    
    def save_xml(self, filename="topology.xml"):
        """保存XML文件"""
        xml_str = ET.tostring(self.xml, encoding='unicode')
        
        # 格式化XML
        try:
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ")
            
            with open(filename, 'w') as f:
                f.write(pretty_xml)
            print(f"Topology saved to {filename}")
            return True
        except Exception as e:
            print(f"Error saving XML: {e}")
            # 保存未格式化的版本
            with open(filename, 'w') as f:
                f.write(xml_str)
            print(f"Topology saved (unformatted) to {filename}")
            return True
    
    def print_statistics(self):
        """打印拓扑统计信息"""
        print("\n" + "="*50)
        print("NR NTN GEO Satellite Topology Statistics")
        print("="*50)
        print(f"Satellite Height: {self.satellite_height/1000:.0f} km (GEO)")
        print(f"Number of Beams: {self.num_beams}")
        print(f"Frequency Reuse: 7-color ({self.num_beams} sectors)")
        print(f"Beam Radius: {self.beam_radius/1000:.0f} km")
        print(f"Total UEs: {len(self.ue_positions)}")
        
        # 每个波束UE数统计
        beam_counts = {}
        for ue in self.ue_positions:
            beam_id = ue['beam_id']
            beam_counts[beam_id] = beam_counts.get(beam_id, 0) + 1
            
        print("\nUEs per Beam:")
        for beam_id in sorted(beam_counts.keys()):
            color = self.beam_colors[beam_id % 7]
            print(f"  Beam {beam_id} ({color}): {beam_counts[beam_id]} UEs")
            
        print("\nFrequency Allocation (7-color reuse):")
        for i in range(7):
            print(f"  Color {i} ({self.beam_colors[i]}): RBs = 160/7 ≈ {160//7} PRBs per beam")
            
        print("="*50 + "\n")


def generate_ascii_topology():
    """生成ASCII艺术拓扑图"""
    print("""
    NR NTN GEO Satellite Topology (ASCII)
    =============================================
    
                    [GEO Satellite]
                     H = 35786 km
                          |
                          | Feeder Link
                          |
                    [Gateway]
                          |
        ----------------------------------------
        |        |        |        |            
      Beam0    Beam1    Beam2    Beam3  ...
     (Red)   (Green)  (Blue)  (Yellow)
        |        |        |        |
       UEs     UEs     UEs     UEs
       
    7-Color Frequency Reuse:
    =========================
    Color 0: Red     -> Beam 0, 7, 14...
    Color 1: Green   -> Beam 1, 8, 15...
    Color 2: Blue    -> Beam 2, 9, 16...
    Color 3: Yellow  -> Beam 3, 10, 17...
    Color 4: Purple  -> Beam 4, 11, 18...
    Color 5: Cyan    -> Beam 5, 12, 19...
    Color 6: Orange  -> Beam 6, 13, 20...
    
    RB Allocation (30 MHz, SCS=15 kHz):
    ===================================
    Total RBs: 160 (DL) / 50 (UL, limited)
    RBs per Color (1/7 reuse): ≈23 RBs
    Reuse Gain: ~8.45 dB
    
    =============================================
    """)


def main():
    print("NR NTN GEO Satellite Topology Generator")
    print("="*50)
    
    # 创建拓扑生成器
    generator = TopologyGenerator()
    
    # 生成UE位置
    generator.generate_ue_positions(ues_per_beam=3)
    
    # 打印统计信息
    generator.print_statistics()
    
    # 打印ASCII拓扑
    generate_ascii_topology()
    
    # 生成XML
    generator.generate_topology_xml()
    
    # 保存XML
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(script_dir, "..", "..", "..", "contrib", "geo-sat", "examples", "topology.xml")
    generator.save_xml(xml_path)
    
    print(f"\nTo view topology:")
    print(f"  1. Open NetAnim")
    print(f"  2. Load {xml_path}")
    print(f"  3. Run simulation to see links and animation")


if __name__ == "__main__":
    main()
