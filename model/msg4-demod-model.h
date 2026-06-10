// SPDX-License-Identifier: GPL-2.0-only
//
// Msg4DemodModel — 系统级 Msg4 PDCCH/PDSCH 解调误码模型 (与 PrachDetectionModel 同款骨架)
//
// 不做波形/链路级译码,只把 (per-UE SNR) 映射成 PDCCH/PDSCH 的 BLER, 用伯努利
// 模拟两次解码(都成功才算 Msg4 解出)。曲线优先级:
//   ① CSV 加载 (内置或外部链路级标定数据) -> 线性插值
//   ② 固定 BLER (SetFixedBler 回退)
//
// 设计:
//   * PDCCH 与 PDSCH 各持一条 BLER-SNR 曲线 (相互独立, 联合成败由 DecodeMsg4 组合)。
//   * 接口刻意做成"接他们组链路级数据零改动":他们给 CSV 即可热插拔。
//
// 设计文档: contrib/geo-sat/Msg4_BLER_SNR_实现方案.docx

#ifndef MSG4_DEMOD_MODEL_H
#define MSG4_DEMOD_MODEL_H

#include "ns3/object.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3 {

/**
 * \brief Msg4 解调统计 (聚合多个 UE 的尝试统计与按 SNR 分箱)。
 */
struct Msg4DemodStats
{
  uint64_t attempts {0};        ///< 总解调尝试 (= 收到的 Msg4 数)
  uint64_t pdcchFail {0};       ///< PDCCH 解码失败次数 (PDSCH 不再尝试)
  uint64_t pdschFail {0};       ///< PDCCH 通过但 PDSCH 失败
  uint64_t success {0};         ///< 两次伯努利都通过
  /// 按 SNR (整数 dB) 分箱: 尝试总数 / 最终成功数 / PDCCH 与 PDSCH 单边失败数
  std::map<int, uint64_t> attemptsBySnrBin;
  std::map<int, uint64_t> successBySnrBin;
  std::map<int, uint64_t> pdcchFailBySnrBin;
  std::map<int, uint64_t> pdschFailBySnrBin;
};

class Msg4DemodModel : public Object
{
public:
  static TypeId GetTypeId ();

  Msg4DemodModel ();
  ~Msg4DemodModel () override;

  // ---- 开关 ----
  void SetEnabled (bool enabled);
  bool IsEnabled () const;

  // ---- 曲线加载: 优先级 CSV > 固定 BLER ----
  /**
   * \brief 用固定 BLER (与 SNR 无关) 替代曲线。
   * 当任一信道未加载曲线时,该信道走固定值。
   */
  void SetFixedBler (double pdcchBler, double pdschBler);

  /// CSV 格式: 跳过 '#' 注释; 表头任意; 每行 "snr_db,bler" (额外列被忽略)。
  bool LoadPdcchCurveCsv (const std::string& path);
  bool LoadPdschCurveCsv (const std::string& path);

  void ClearPdcchCurve ();
  void ClearPdschCurve ();

  // ---- 查询: 经线性插值 (边界钳位) ----
  double GetPdcchBler (double snrDb) const;
  double GetPdschBler (double snrDb) const;

  // ---- 核心: 两次独立伯努利, 都成功才返回 true; 同时记统计 ----
  bool DecodeMsg4 (double snrDb);

  // ---- 统计 ----
  Msg4DemodStats GetStats () const;
  void ResetStats ();

  int64_t AssignStreams (int64_t stream);

private:
  struct CurvePoint
  {
    double snrDb;
    double bler;
  };

  /// CSV → vector<CurvePoint>, 按 snr 排序; 返回是否成功。
  static bool ParseCsv (const std::string& path, std::vector<CurvePoint>& out);
  /// 线性插值 (snr 越界时钳位到端点 bler)。
  static double Interpolate (const std::vector<CurvePoint>& curve, double snrDb);
  static double Clamp01 (double v);
  static int SnrBin (double snrDb);
  /// 以概率 p 返回 true (p<=0 → false; p>=1 → true)。
  bool DrawBernoulli (double p);

  bool m_enabled {true};

  std::vector<CurvePoint> m_pdcchCurve;   ///< 空 → 用 m_pdcchFixed
  std::vector<CurvePoint> m_pdschCurve;
  double m_pdcchFixed {0.0};               ///< 默认不丢
  double m_pdschFixed {0.0};

  Ptr<UniformRandomVariable> m_rng;
  Msg4DemodStats m_stats;
};

} // namespace ns3

#endif // MSG4_DEMOD_MODEL_H
