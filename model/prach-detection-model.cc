/*
 * ============================================================================
 *  PrachDetectionModel —— 系统级 NR PRACH 虚警/漏检模型
 * ============================================================================
 *
 * 【定位】这是一个"系统级"模型, 不做波形相关 (不生成 Zadoff-Chu 序列、不做匹配
 *         滤波、没有判决门限)。它只把一个 PRACH 的 SNR 映射成两个概率:
 *           - Pd  (Detection Probability)   : 有 UE 发的前导码被正确检出的概率
 *           - Pfa (False-Alarm Probability) : 空前导码被误判为"有信号"的概率
 *         再用伯努利抽样决定本次到底"检到 / 漏检 / 虚警"。
 *
 * 【概率从哪来】两种配置方式 (二选一):
 *   1. 固定值: SetFixedProbabilities(pd, pfa) —— 与 SNR 无关的常数。
 *   2. SNR 曲线: LoadCurveCsv(path) —— 加载链路级仿真/3GPP 给出的 Pd/Pfa-SNR
 *      曲线 (CSV: snr_db,pd,pfa), 运行时按当前 SNR 线性插值。曲线非空时优先用曲线。
 *
 * 【谁来调用】调度器 GeoBeamScheduler::ProcessPrachWindow() 在每个 PRACH 聚合
 *   窗口结束时:
 *     - 对每个"有 UE 的前导码组": DetectActivePreamble(snr) 决定是否发 RAR;
 *     - 对每个"空闲前导码":       DetectFalseAlarm(snr)   决定是否产生幽灵 RAR。
 *   并通过 Record* 系列累计统计量 (PrachDetectionStats), 供仿真结束后导出。
 *
 * 【可复现性】检测抽样用 ns-3 的 UniformRandomVariable; AssignStreams() 接入
 *   RngRun 体系, 因此同一 RngRun 结果可复现, 不同 RngRun 可做蒙特卡洛平均。
 * ============================================================================
 */
#include "prach-detection-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PrachDetectionModel");
NS_OBJECT_ENSURE_REGISTERED (PrachDetectionModel);

// TypeId: 注册三个可配置属性 (Enabled / 固定 Pd / 固定 Pfa), 便于用
// Config::SetDefault 或 attribute 路径调整, 不必改代码。
TypeId
PrachDetectionModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::PrachDetectionModel")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<PrachDetectionModel> ()
    .AddAttribute ("Enabled",
                   "Enable system-level PRACH missed-detection and false-alarm sampling.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PrachDetectionModel::m_enabled),
                   MakeBooleanChecker ())
    .AddAttribute ("FixedDetectionProbability",
                   "Fixed active-preamble detection probability used when no CSV curve is loaded.",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&PrachDetectionModel::m_fixedPd),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("FixedFalseAlarmProbability",
                   "Fixed empty-preamble false-alarm probability used when no CSV curve is loaded.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&PrachDetectionModel::m_fixedPfa),
                   MakeDoubleChecker<double> (0.0, 1.0));
  return tid;
}

// 默认: 关闭检测误差 (Pd=1, Pfa=0 → 等价"理想检测"), 曲线为空。
PrachDetectionModel::PrachDetectionModel ()
  : m_enabled (false),
    m_fixedPd (1.0),
    m_fixedPfa (0.0),
    m_rng (CreateObject<UniformRandomVariable> ())
{
}

PrachDetectionModel::~PrachDetectionModel () = default;

// 总开关: 关闭时调度器会跳过所有 Detect*/Record* 调用 (视为理想检测)。
void
PrachDetectionModel::SetEnabled (bool enabled)
{
  m_enabled = enabled;
}

bool
PrachDetectionModel::IsEnabled () const
{
  return m_enabled;
}

// 设定与 SNR 无关的固定 Pd/Pfa。注意会 ClearCurve(): 固定值与曲线互斥,
// 设固定值即放弃曲线 (后续 Get*Probability 因曲线空而回落到固定值)。
void
PrachDetectionModel::SetFixedProbabilities (double pd, double pfa)
{
  m_fixedPd = ClampProbability (pd);
  m_fixedPfa = ClampProbability (pfa);
  ClearCurve ();
}

// 从 CSV 加载 Pd/Pfa-SNR 曲线。每行格式: snr_db,pd,pfa (逗号或空格分隔,
// '#' 开头为注释)。逐行解析后按 SNR 升序排序 (插值要求有序)。
// 返回 false: 文件打不开 或 没有任何有效行 (此时不改动现有曲线)。
bool
PrachDetectionModel::LoadCurveCsv (const std::string& path)
{
  std::ifstream input (path);
  if (!input.is_open ())
    {
      NS_LOG_ERROR ("Cannot open PRACH detection curve CSV: " << path);
      return false;
    }

  std::vector<CurvePoint> parsed;
  std::string line;
  while (std::getline (input, line))
    {
      CurvePoint point;
      if (TryParseCurveLine (line, point))   // 跳过空行/注释/格式错误行
        {
          parsed.push_back (point);
        }
    }

  if (parsed.empty ())
    {
      NS_LOG_ERROR ("PRACH detection curve CSV has no valid rows: " << path);
      return false;
    }

  // 按 SNR 升序排序: Interpolate() 依赖单调有序的采样点做线性插值。
  std::sort (parsed.begin (), parsed.end (),
             [] (const CurvePoint& a, const CurvePoint& b) {
               return a.snrDb < b.snrDb;
             });
  m_curve = parsed;
  return true;
}

// 清空曲线 → 回落到固定 Pd/Pfa 模式。
void
PrachDetectionModel::ClearCurve ()
{
  m_curve.clear ();
}

// 取给定 SNR 下的检测概率 Pd: 曲线非空则插值, 否则用固定值。
double
PrachDetectionModel::GetDetectionProbability (double snrDb) const
{
  if (m_curve.empty ())
    {
      return ClampProbability (m_fixedPd);
    }
  return Interpolate (snrDb, true);
}

// 取给定 SNR 下的虚警概率 Pfa: 曲线非空则插值, 否则用固定值。
double
PrachDetectionModel::GetFalseAlarmProbability (double snrDb) const
{
  if (m_curve.empty ())
    {
      return ClampProbability (m_fixedPfa);
    }
  return Interpolate (snrDb, false);
}

// 判定"有 UE 的前导码"是否被检出: 以 Pd(snr) 做伯努利抽样。
// 返回 true=检到 (调度器发 RAR), false=漏检 (不发 RAR)。
bool
PrachDetectionModel::DetectActivePreamble (double snrDb)
{
  return DrawBernoulli (GetDetectionProbability (snrDb));
}

// 判定"空闲前导码"是否产生虚警: 以 Pfa(snr) 做伯努利抽样。
// 返回 true=虚警 (调度器为不存在的 UE 发幽灵 RAR、白占 Msg3 资源)。
bool
PrachDetectionModel::DetectFalseAlarm (double snrDb)
{
  return DrawBernoulli (GetFalseAlarmProbability (snrDb));
}

// ---- 统计累计 (一个"组" = 一个 (RA-RNTI, preambleId) 检测试验) ----
// 注: 第一个参数 ueAttempts (组内碰撞 UE 数) 在 active 这里不用, 仅 missed 用;
//     此处同时把该组计入对应 SNR 分箱, 供事后画"实测 Pd-SNR"曲线。
void
PrachDetectionModel::RecordActiveGroup (uint32_t, double snrDb)
{
  ++m_stats.activePreambleGroups;
  ++m_stats.activeGroupsBySnrBin[SnrBin (snrDb)];
}

// 记录一个被成功检出的组 (同时计入 SNR 分箱的 detected 计数)。
void
PrachDetectionModel::RecordDetectedGroup (double snrDb)
{
  ++m_stats.detectedGroups;
  ++m_stats.detectedGroupsBySnrBin[SnrBin (snrDb)];
}

// 记录一个被漏检的组。ueAttempts = 该组里碰撞的 UE 数, 累加到 missedUeAttempts,
// 以便区分"按组算"和"按 UE 算"的漏检影响 (一个组漏检会连累组内所有 UE)。
void
PrachDetectionModel::RecordMissedGroup (uint32_t ueAttempts)
{
  ++m_stats.missedGroups;
  m_stats.missedUeAttempts += ueAttempts;
}

// 记录一次"空前导码虚警试验"(无论是否触发虚警)。这是 Pfa 的分母:
// 实测 Pfa = falseAlarmGroups / falseAlarmTrials。
void
PrachDetectionModel::RecordFalseAlarmTrial ()
{
  ++m_stats.falseAlarmTrials;
}

// 记录一次真正触发的虚警, 并累加它白白占用的 Msg3 资源块数 (评估资源浪费)。
void
PrachDetectionModel::RecordFalseAlarmGroup (uint32_t msg3GrantRbs)
{
  ++m_stats.falseAlarmGroups;
  m_stats.falseAlarmMsg3GrantRbs += msg3GrantRbs;
}

// 把 SNR 量化到最近的整数 dB, 作为统计直方图的 key (1 dB 分箱)。
// 非有限值 (NaN/Inf) 归到 0 箱, 避免污染 map。
int
PrachDetectionModel::SnrBin (double snrDb)
{
  if (!std::isfinite (snrDb))
    {
      return 0;
    }
  return static_cast<int> (std::lround (snrDb));   // 1 dB 分箱
}

// 返回统计快照 (含分箱直方图, 按值拷贝)。
PrachDetectionStats
PrachDetectionModel::GetStats () const
{
  return m_stats;
}

// 清零全部统计 (含分箱 map): 多次独立运行/批次之间调用以避免累加污染。
void
PrachDetectionModel::ResetStats ()
{
  m_stats = PrachDetectionStats ();
}

// 接入 ns-3 随机流体系: 指定本模型 RNG 的 stream, 使检测抽样可随 RngRun 复现。
int64_t
PrachDetectionModel::AssignStreams (int64_t stream)
{
  m_rng->SetStream (stream);
  return 1;
}

// 把概率夹到 [0,1]; 非有限值视为 0。所有对外暴露的概率都经它过滤, 保证合法。
double
PrachDetectionModel::ClampProbability (double value)
{
  if (!std::isfinite (value))
    {
      return 0.0;
    }
  return std::min (1.0, std::max (0.0, value));
}

// 解析 CSV 一行 → CurvePoint。规则: 去掉行首空白; 空行或 '#' 开头 → 跳过 (返回
// false); 逗号统一替换成空格后, 依次读 snr/pd/pfa 三个 double, 不足三列 → 跳过。
bool
PrachDetectionModel::TryParseCurveLine (const std::string& line, CurvePoint& point)
{
  std::string trimmed = line;
  trimmed.erase (trimmed.begin (),
                 std::find_if (trimmed.begin (), trimmed.end (),
                               [] (unsigned char ch) { return !std::isspace (ch); }));
  if (trimmed.empty () || trimmed[0] == '#')
    {
      return false;
    }

  std::replace (trimmed.begin (), trimmed.end (), ',', ' ');
  std::istringstream iss (trimmed);
  double snr;
  double pd;
  double pfa;
  if (!(iss >> snr >> pd >> pfa))
    {
      return false;
    }

  point.snrDb = snr;
  point.pd = ClampProbability (pd);
  point.pfa = ClampProbability (pfa);
  return true;
}

// 在曲线上对给定 SNR 做线性插值。detectionProbability=true 取 Pd 列, false 取 Pfa 列。
// 边界处理: 低于曲线最小 SNR → 取首点; 高于最大 SNR → 取末点 (即两端做"钳位"外推,
// 不外插)。中间则在相邻两点间线性内插。
double
PrachDetectionModel::Interpolate (double snrDb, bool detectionProbability) const
{
  if (m_curve.empty ())
    {
      return detectionProbability ? ClampProbability (m_fixedPd) : ClampProbability (m_fixedPfa);
    }

  if (snrDb <= m_curve.front ().snrDb)   // 低于下界: 钳到首点
    {
      return detectionProbability ? m_curve.front ().pd : m_curve.front ().pfa;
    }
  if (snrDb >= m_curve.back ().snrDb)     // 高于上界: 钳到末点
    {
      return detectionProbability ? m_curve.back ().pd : m_curve.back ().pfa;
    }

  // 找到第一个 snrDb <= hi.snrDb 的区间 [lo, hi], 线性内插。
  for (std::size_t i = 1; i < m_curve.size (); ++i)
    {
      const CurvePoint& hi = m_curve[i];
      if (snrDb <= hi.snrDb)
        {
          const CurvePoint& lo = m_curve[i - 1];
          const double span = hi.snrDb - lo.snrDb;
          const double t = (span > 0.0) ? ((snrDb - lo.snrDb) / span) : 0.0;
          const double loValue = detectionProbability ? lo.pd : lo.pfa;
          const double hiValue = detectionProbability ? hi.pd : hi.pfa;
          return ClampProbability (loValue + t * (hiValue - loValue));
        }
    }

  return detectionProbability ? m_curve.back ().pd : m_curve.back ().pfa;
}

// 伯努利抽样: 以概率 p 返回 true。p<=0 必假, p>=1 必真 (省一次取随机数),
// 否则抽 [0,1) 均匀随机数与 p 比较。这是漏检/虚警随机性的唯一来源。
bool
PrachDetectionModel::DrawBernoulli (double probability)
{
  const double p = ClampProbability (probability);
  if (p <= 0.0)
    {
      return false;
    }
  if (p >= 1.0)
    {
      return true;
    }
  return m_rng->GetValue (0.0, 1.0) < p;
}

} // namespace ns3
