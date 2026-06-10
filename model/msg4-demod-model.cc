// SPDX-License-Identifier: GPL-2.0-only
//
// Msg4DemodModel 实现 (镜像 PrachDetectionModel 的 CSV/插值/Bernoulli/SnrBin 模式)

#include "msg4-demod-model.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Msg4DemodModel");
NS_OBJECT_ENSURE_REGISTERED (Msg4DemodModel);

TypeId
Msg4DemodModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::Msg4DemodModel")
                          .SetParent<Object> ()
                          .SetGroupName ("GeoSat")
                          .AddConstructor<Msg4DemodModel> ();
  return tid;
}

Msg4DemodModel::Msg4DemodModel ()
{
  m_rng = CreateObject<UniformRandomVariable> ();
}

Msg4DemodModel::~Msg4DemodModel () = default;

void
Msg4DemodModel::SetEnabled (bool enabled)
{
  m_enabled = enabled;
}

bool
Msg4DemodModel::IsEnabled () const
{
  return m_enabled;
}

void
Msg4DemodModel::SetFixedBler (double pdcchBler, double pdschBler)
{
  m_pdcchFixed = Clamp01 (pdcchBler);
  m_pdschFixed = Clamp01 (pdschBler);
}

double
Msg4DemodModel::Clamp01 (double v)
{
  if (v < 0.0)
    return 0.0;
  if (v > 1.0)
    return 1.0;
  return v;
}

int
Msg4DemodModel::SnrBin (double snrDb)
{
  return static_cast<int> (std::lround (snrDb));
}

// CSV: 跳过 '#' 注释行; 支持表头(任意名);第一/第二列分别取 snr 与 bler。
bool
Msg4DemodModel::ParseCsv (const std::string& path, std::vector<CurvePoint>& out)
{
  std::ifstream f (path);
  if (!f.is_open ())
    {
      NS_LOG_WARN ("Msg4DemodModel: 无法打开曲线 CSV: " << path);
      return false;
    }
  out.clear ();
  std::string line;
  bool headerSkipped = false;
  while (std::getline (f, line))
    {
      // 去首尾空白
      size_t start = line.find_first_not_of (" \t\r\n");
      if (start == std::string::npos)
        continue;
      line = line.substr (start);
      if (line.empty () || line[0] == '#')
        continue;
      // 替换逗号为空格便于 stream 解析,同时兼容 tab/空格分隔
      for (char& c : line)
        {
          if (c == ',' || c == '\t')
            c = ' ';
        }
      std::istringstream iss (line);
      double snr = 0.0, bler = 0.0;
      if (!(iss >> snr >> bler))
        {
          // 第一行可能是表头如 "snr_db,bler" → 跳过一次
          if (!headerSkipped)
            {
              headerSkipped = true;
              continue;
            }
          NS_LOG_WARN ("Msg4DemodModel: 解析不出 (snr,bler): '" << line << "' @ " << path);
          continue;
        }
      out.push_back ({snr, Clamp01 (bler)});
    }
  std::sort (out.begin (), out.end (),
             [] (const CurvePoint& a, const CurvePoint& b) { return a.snrDb < b.snrDb; });
  return !out.empty ();
}

double
Msg4DemodModel::Interpolate (const std::vector<CurvePoint>& curve, double snrDb)
{
  if (curve.empty ())
    return 0.0;
  if (snrDb <= curve.front ().snrDb)
    return curve.front ().bler;
  if (snrDb >= curve.back ().snrDb)
    return curve.back ().bler;
  // 二分定位区间, 线性插值
  auto it = std::upper_bound (
      curve.begin (), curve.end (), snrDb,
      [] (double v, const CurvePoint& p) { return v < p.snrDb; });
  auto hi = it;
  auto lo = std::prev (it);
  double t = (snrDb - lo->snrDb) / (hi->snrDb - lo->snrDb);
  return Clamp01 (lo->bler + t * (hi->bler - lo->bler));
}

bool
Msg4DemodModel::LoadPdcchCurveCsv (const std::string& path)
{
  return ParseCsv (path, m_pdcchCurve);
}

bool
Msg4DemodModel::LoadPdschCurveCsv (const std::string& path)
{
  return ParseCsv (path, m_pdschCurve);
}

void
Msg4DemodModel::ClearPdcchCurve ()
{
  m_pdcchCurve.clear ();
}

void
Msg4DemodModel::ClearPdschCurve ()
{
  m_pdschCurve.clear ();
}

double
Msg4DemodModel::GetPdcchBler (double snrDb) const
{
  return m_pdcchCurve.empty () ? m_pdcchFixed : Interpolate (m_pdcchCurve, snrDb);
}

double
Msg4DemodModel::GetPdschBler (double snrDb) const
{
  return m_pdschCurve.empty () ? m_pdschFixed : Interpolate (m_pdschCurve, snrDb);
}

bool
Msg4DemodModel::DrawBernoulli (double p)
{
  if (p <= 0.0)
    return false;  // 0 失败概率 → 不"失败" (这里 p 是"失败"的概率, 见 DecodeMsg4)
  if (p >= 1.0)
    return true;
  return m_rng->GetValue (0.0, 1.0) < p;
}

bool
Msg4DemodModel::DecodeMsg4 (double snrDb)
{
  int bin = SnrBin (snrDb);
  m_stats.attempts++;
  m_stats.attemptsBySnrBin[bin]++;
  if (!m_enabled)
    {
      m_stats.success++;
      m_stats.successBySnrBin[bin]++;
      return true;
    }
  // ① PDCCH: 以 BLER 概率失败
  double pdcchBler = GetPdcchBler (snrDb);
  if (DrawBernoulli (pdcchBler))
    {
      m_stats.pdcchFail++;
      m_stats.pdcchFailBySnrBin[bin]++;
      return false;
    }
  // ② PDSCH: 以 BLER 概率失败
  double pdschBler = GetPdschBler (snrDb);
  if (DrawBernoulli (pdschBler))
    {
      m_stats.pdschFail++;
      m_stats.pdschFailBySnrBin[bin]++;
      return false;
    }
  m_stats.success++;
  m_stats.successBySnrBin[bin]++;
  return true;
}

Msg4DemodStats
Msg4DemodModel::GetStats () const
{
  return m_stats;
}

void
Msg4DemodModel::ResetStats ()
{
  m_stats = Msg4DemodStats ();
}

int64_t
Msg4DemodModel::AssignStreams (int64_t stream)
{
  m_rng->SetStream (stream);
  return 1;
}

} // namespace ns3
