#include "sat-channel.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/phased-array-model.h"
#include "ns3/pointer.h"
#include <cmath>
#include <map>
#include <array>
#include <vector>
#include <complex>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatChannel");

NS_OBJECT_ENSURE_REGISTERED (SatChannel);

// ===========================================================================
// 第一部分：NTN 参数表与数据结构 (源自 3GPP TR 38.811)
// ===========================================================================

// 参数索引枚举
enum Table3gppParams {
    uLgDS, sigLgDS, uLgASD, sigLgASD, uLgASA, sigLgASA, uLgZSA, sigLgZSA, uLgZSD, sigLgZSD,
    uK, sigK, rTau, uXpr, sigXpr, numOfCluster, raysPerCluster, cDS, cASD, cASA, cZSA,
    perClusterShadowingStd
};

// 空间相关性矩阵: NTN Urban LOS
static constexpr std::array<std::array<double, 7>, 7> sqrtC_NTN_Urban_LOS = {{
    {1, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0, 0},
    {-0.4, -0.4, 0.824621, 0, 0, 0, 0},
    {-0.5, 0, 0.242536, 0.83137, 0, 0, 0},
    {-0.5, -0.2, 0.630593, -0.484671, 0.278293, 0, 0},
    {0, 0, -0.242536, 0.672172, 0.642214, 0.27735, 0},
    {-0.8, 0, -0.388057, -0.367926, 0.238537, 0, 0.130931},
}};

// 完整数据表: NTN Urban LOS (包含 S 和 Ka 频段所有仰角)
static const std::map<std::string, std::map<int, std::array<double, 22>>> NTNUrbanLOS = {
    {"S", {
         {10, {-7.97, 1.0, -2.6, 0.79, 0.18, 0.74, -0.63, 2.6, -2.54, 2.62, 31.83, 13.84, 2.5, 8.0, 4.0, 4.0, 20.0, 3.9, 0.09, 12.55, 1.25, 3.0}},
         {20, {-8.12, 0.83, -2.48, 0.8, 0.42, 0.9, -0.15, 3.31, -2.67, 2.96, 18.78, 13.78, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.09, 12.76, 3.23, 3.0}},
         {30, {-8.21, 0.68, -2.44, 0.91, 0.41, 1.3, 0.54, 1.1, -2.03, 0.86, 10.49, 10.42, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.12, 14.36, 4.39, 3.0}},
         {40, {-8.31, 0.48, -2.6, 1.02, 0.18, 1.69, 0.35, 1.59, -2.28, 1.19, 7.46, 8.01, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.16, 16.42, 5.72, 3.0}},
         {50, {-8.37, 0.38, -2.71, 1.17, -0.07, 2.04, 0.27, 1.62, -2.48, 1.4, 6.52, 8.27, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.2, 17.13, 6.17, 3.0}},
         {60, {-8.39, 0.24, -2.76, 1.17, -0.43, 2.54, 0.26, 0.97, -2.56, 0.85, 5.47, 7.26, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.28, 19.01, 7.36, 3.0}},
         {70, {-8.38, 0.18, -2.78, 1.2, -0.64, 2.47, -0.12, 1.99, -2.96, 1.61, 4.54, 5.53, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.44, 19.31, 7.3, 3.0}},
         {80, {-8.35, 0.13, -2.65, 1.45, -0.91, 2.69, -0.21, 1.82, -3.08, 1.49, 4.03, 4.49, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 0.9, 22.39, 7.7, 3.0}},
         {90, {-8.34, 0.09, -2.27, 1.85, -0.54, 1.66, -0.07, 1.43, -3.0, 1.09, 3.68, 3.14, 2.5, 8.0, 4.0, 3.0, 20.0, 3.9, 2.87, 27.8, 9.25, 3.0}},
     }},
    {"Ka", {
         {10, {-8.52, 0.92, -3.18, 0.79, -0.4, 0.77, -0.67, 2.22, -2.61, 2.41, 40.18, 16.99, 2.5, 8.0, 4.0, 4.0, 20.0, 1.6, 0.09, 11.8, 1.14, 3.0}},
         {20, {-8.59, 0.79, -3.05, 0.87, -0.15, 0.97, -0.34, 3.04, -2.82, 2.59, 23.62, 18.96, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.09, 11.6, 2.78, 3.0}},
         {30, {-8.51, 0.65, -2.98, 1.04, -0.18, 1.58, 0.07, 1.33, -2.48, 1.02, 12.48, 14.23, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.11, 13.05, 3.87, 3.0}},
         {40, {-8.49, 0.48, -3.11, 1.06, -0.31, 1.69, -0.08, 1.45, -2.76, 1.27, 8.56, 11.06, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.15, 14.56, 4.94, 3.0}},
         {50, {-8.48, 0.46, -3.19, 1.12, -0.58, 2.13, -0.21, 1.62, -2.93, 1.38, 7.42, 11.21, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.18, 15.35, 5.41, 3.0}},
         {60, {-8.44, 0.34, -3.25, 1.14, -0.9, 2.51, -0.25, 1.06, -3.05, 0.96, 5.97, 9.47, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.27, 16.97, 6.31, 3.0}},
         {70, {-8.4, 0.27, -3.33, 1.25, -1.16, 2.47, -0.61, 1.88, -3.45, 1.51, 4.88, 7.24, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.42, 17.96, 6.66, 3.0}},
         {80, {-8.37, 0.19, -3.22, 1.35, -1.48, 2.61, -0.79, 1.87, -3.66, 1.49, 4.22, 5.79, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 0.86, 20.68, 7.31, 3.0}},
         {90, {-8.35, 0.14, -2.83, 1.62, -1.14, 1.7, -0.58, 1.19, -3.56, 0.89, 3.81, 4.25, 2.5, 8.0, 4.0, 3.0, 20.0, 1.6, 2.55, 25.08, 9.23, 3.0}},
     }},
};

// ===========================================================================
// 第二部分：SatChannel 类实现
// ===========================================================================

TypeId
SatChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatChannel")
    .SetParent<ThreeGppChannelModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<SatChannel> ();
  return tid;
}

SatChannel::SatChannel ()
{
}

SatChannel::~SatChannel ()
{
}

// ---------------------------------------------------------------------------
// 辅助函数：计算 FSPL
// ---------------------------------------------------------------------------
double
SatChannel::CalcFspl (double distance3D, double frequencyHz) const
{
  if (distance3D < 1.0) distance3D = 1.0; 
  // FSPL(dB) = 20log10(d) + 20log10(f) - 147.55
  return 20 * std::log10 (distance3D) + 20 * std::log10 (frequencyHz) - 147.55;
}

// ---------------------------------------------------------------------------
// 辅助函数：计算多波束增益
// ---------------------------------------------------------------------------
double
SatChannel::CalcBeamGain (Ptr<const MobilityModel> satMob, Ptr<const MobilityModel> ueMob) const
{
  // 1. 获取距离和高度差
  double dist = satMob->GetDistanceFrom (ueMob);
  double dz = std::abs(satMob->GetPosition ().z - ueMob->GetPosition ().z);
  
  // 2. 计算偏轴角 (Off-boresight angle)
  double theta = 0.0;
  if (dist > 0)
    {
      theta = std::acos (dz / dist); // 结果为弧度
    }

  // 3. 定义波束参数
  double maxGain = 45.0; // 主瓣增益 dBi
  double beamWidthRad = 2.0 * M_PI / 180.0; // 3dB 波束宽度
  
  // 4. 计算抛物面天线增益
  // G(theta) = Gmax - 12 * (theta / theta_3dB)^2
  double gain = maxGain - 12.0 * std::pow (theta / beamWidthRad, 2);
  
  // 5. 旁瓣干扰地板
  double sideLobeLevel = -5.0; // dBi
  return std::max (gain, sideLobeLevel);
}

// ---------------------------------------------------------------------------
// [核心函数] 物理层功率计算
// ---------------------------------------------------------------------------
double 
SatChannel::DoRxPowerCalculation (Ptr<const MobilityModel> txMobility,
                                  Ptr<const MobilityModel> rxMobility,
                                  Ptr<const PhasedArrayModel> txAntenna,
                                  Ptr<const PhasedArrayModel> rxAntenna,
                                  double txPowerDbm) const
{
  // 1. 获取位置 (MobilityModel 第一步)
  double distance = txMobility->GetDistanceFrom (rxMobility);
  double freq = GetFrequency ();

  // 2. 计算自由空间损耗 (FSPL)
  double fsplDb = CalcFspl (distance, freq);

  // 3. 叠加多波束干扰/增益模型
  double beamGainDb = 0.0;
  // 判定谁是卫星（高度较高的那个）
  if (txMobility->GetPosition ().z > rxMobility->GetPosition ().z)
  {
      beamGainDb = CalcBeamGain (txMobility, rxMobility); // Tx是卫星
  }
  else
  {
      beamGainDb = CalcBeamGain (rxMobility, txMobility); // Rx是卫星
  }

  // 4. 计算接收功率
  double rxPowerDbm = txPowerDbm + beamGainDb - fsplDb;

  // 输出结果到日志
  NS_LOG_INFO ("[SatChannel::DoRxPowerCalculation] Dist=" << distance/1000.0 << "km"
               << ", FSPL=" << fsplDb << "dB"
               << ", BeamGain=" << beamGainDb << "dBi"
               << " => RxPower=" << rxPowerDbm << "dBm");

  return rxPowerDbm;
}

// ---------------------------------------------------------------------------
// 重写 GetChannel：调用 DoRxPowerCalculation 并应用到矩阵
// ---------------------------------------------------------------------------
Ptr<const MatrixBasedChannelModel::ChannelMatrix>
SatChannel::GetChannel (Ptr<const MobilityModel> aMob,
                        Ptr<const MobilityModel> bMob,
                        Ptr<const PhasedArrayModel> aAntenna,
                        Ptr<const PhasedArrayModel> bAntenna)
{
  // 1. 调用父类生成 3GPP 小尺度衰落矩阵
  auto baseMatrixConst = ThreeGppChannelModel::GetChannel (aMob, bMob, aAntenna, bAntenna);
  
  // 2. 计算大尺度损耗 (Path Loss)
  double txPowerRef = 0.0; // 参考发射功率 0dBm
  double rxPowerRef = DoRxPowerCalculation (aMob, bMob, aAntenna, bAntenna, txPowerRef);
  
  // Total Loss = Tx - Rx
  double totalLossDb = -rxPowerRef;

  // 3. 将大尺度损耗应用到信道矩阵
  // 线性缩放因子: Scale = 10^(-Loss/20)
  double scaleFactor = std::pow (10.0, -totalLossDb / 20.0);

  // 去除 const 以便修改矩阵
  auto matrix = const_cast<MatrixBasedChannelModel::ChannelMatrix*> (PeekPointer (baseMatrixConst));
  
  // 遍历矩阵并缩放
  for (unsigned int p = 0; p < matrix->m_channel.GetNumPages (); ++p) 
    {
      for (unsigned int r = 0; r < matrix->m_channel.GetNumRows (); ++r) 
        {
          for (unsigned int c = 0; c < matrix->m_channel.GetNumCols (); ++c) 
            {
               matrix->m_channel (r, c, p) *= scaleFactor;
            }
        }
    }

  return baseMatrixConst;
}

// ---------------------------------------------------------------------------
// 重写 GetThreeGppTable：NTN 查表逻辑
// ---------------------------------------------------------------------------
Ptr<const ThreeGppChannelModel::ParamsTable>
SatChannel::GetThreeGppTable (Ptr<const ChannelCondition> channelCondition,
                              double hBS,
                              double hUT,
                              double distance2D) const
{
  std::string scenario = GetScenario ();

  // 非 NTN 场景，调用父类逻辑
  if (scenario.find ("NTN") == std::string::npos)
    {
      return ThreeGppChannelModel::GetThreeGppTable (channelCondition, hBS, hUT, distance2D);
    }

  // === NTN 逻辑 ===
  double fcGHz = GetFrequency () / 1.0e9;
  std::string freqBand = (fcGHz < 13) ? "S" : "Ka";

  // 估算仰角
  double heightDiff = std::abs(hBS - hUT);
  double elevAngleDeg = std::atan2 (heightDiff, distance2D) * 180.0 / M_PI;

  // 量化仰角 (10, 20...90)
  int elevAngleQuantized = (elevAngleDeg < 10) ? 10 : std::round (elevAngleDeg / 10.0) * 10;
  if (elevAngleQuantized > 90) elevAngleQuantized = 90;

  Ptr<ParamsTable> table3gpp = Create<ParamsTable> ();
  
  // 查表逻辑 (S或Ka)
  if (NTNUrbanLOS.count(freqBand))
    {
       // 默认取该频段第一个作为 fallback
       const auto& bandMap = NTNUrbanLOS.at(freqBand);
       auto it = bandMap.find(elevAngleQuantized);
       
       // 如果找不到对应角度，取最接近的（这里简单处理为取第一个，实际应插值或取近邻）
       const auto& params = (it != bandMap.end()) ? it->second : bandMap.begin()->second;

       table3gpp->m_uLgDS = params[Table3gppParams::uLgDS];
       table3gpp->m_sigLgDS = params[Table3gppParams::sigLgDS];
       table3gpp->m_uLgASD = params[Table3gppParams::uLgASD];
       table3gpp->m_sigLgASD = params[Table3gppParams::sigLgASD];
       table3gpp->m_uLgASA = params[Table3gppParams::uLgASA];
       table3gpp->m_sigLgASA = params[Table3gppParams::sigLgASA];
       table3gpp->m_uLgZSA = params[Table3gppParams::uLgZSA];
       table3gpp->m_sigLgZSA = params[Table3gppParams::sigLgZSA];
       table3gpp->m_uLgZSD = params[Table3gppParams::uLgZSD];
       table3gpp->m_sigLgZSD = params[Table3gppParams::sigLgZSD];
       table3gpp->m_uK = params[Table3gppParams::uK];
       table3gpp->m_sigK = params[Table3gppParams::sigK];
       table3gpp->m_rTau = params[Table3gppParams::rTau];
       table3gpp->m_uXpr = params[Table3gppParams::uXpr];
       table3gpp->m_sigXpr = params[Table3gppParams::sigXpr];
       table3gpp->m_numOfCluster = (uint8_t)params[Table3gppParams::numOfCluster];
       table3gpp->m_raysPerCluster = (uint8_t)params[Table3gppParams::raysPerCluster];
       table3gpp->m_cDS = params[Table3gppParams::cDS] * 1e-9;
       table3gpp->m_cASD = params[Table3gppParams::cASD];
       table3gpp->m_cASA = params[Table3gppParams::cASA];
       table3gpp->m_cZSA = params[Table3gppParams::cZSA];
       table3gpp->m_perClusterShadowingStd = params[Table3gppParams::perClusterShadowingStd];

       // 填充空间相关矩阵
       for (uint8_t row = 0; row < 7; row++)
           for (uint8_t column = 0; column < 7; column++)
               table3gpp->m_sqrtC[row][column] = sqrtC_NTN_Urban_LOS[row][column];
    }
  else
    {
      // 兜底：未定义频段则回退父类
      return ThreeGppChannelModel::GetThreeGppTable (channelCondition, hBS, hUT, distance2D);
    }

  return table3gpp;
}

// ===========================================================================
// qyh 增量: FsplDb / BeamGainDbi / ParabolicGainDbi
// ===========================================================================

double
SatChannel::FsplDb (double distance3D, double frequencyHz) const
{
  return CalcFspl (distance3D, frequencyHz);
}

double
SatChannel::ParabolicGainDbi (double thetaRad, double theta3dbRad,
                              double maxGainDbi, double sideLobeDbi)
{
  if (theta3dbRad <= 0.0) return maxGainDbi;
  double gain = maxGainDbi - 12.0 * std::pow (thetaRad / theta3dbRad, 2.0);
  return std::max (gain, sideLobeDbi);
}

double
SatChannel::BeamGainDbi (const Vector& satPos, const Vector& uePos,
                         const Vector& beamCenter, double theta3dbRad) const
{
  Vector satToBeam (beamCenter.x - satPos.x, beamCenter.y - satPos.y, beamCenter.z - satPos.z);
  Vector satToUe   (uePos.x - satPos.x,       uePos.y - satPos.y,       uePos.z - satPos.z);
  double dotp = satToBeam.x * satToUe.x + satToBeam.y * satToUe.y + satToBeam.z * satToUe.z;
  double nb = std::sqrt (satToBeam.x * satToBeam.x + satToBeam.y * satToBeam.y
                         + satToBeam.z * satToBeam.z);
  double nu = std::sqrt (satToUe.x * satToUe.x + satToUe.y * satToUe.y
                         + satToUe.z * satToUe.z);
  double cosTheta = (nb > 0 && nu > 0)
                    ? std::max (-1.0, std::min (1.0, dotp / (nb * nu))) : 1.0;
  double theta = std::acos (cosTheta);
  return ParabolicGainDbi (theta, theta3dbRad, 45.0, -5.0);
}

} // namespace ns3