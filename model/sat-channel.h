#ifndef SAT_CHANNEL_H
#define SAT_CHANNEL_H

#include "ns3/three-gpp-channel-model.h"
#include "ns3/mobility-model.h"
#include "ns3/phased-array-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/ptr.h"

namespace ns3 {

/**
 * \brief 卫星信道模型 (SatChannel)
 * \details 继承自 ThreeGppChannelModel (Old架构)，集成了 NTN 数据表与物理层功率计算。
 */
class SatChannel : public ThreeGppChannelModel
{
public:
  /**
   * \brief 获取类型ID
   */
  static TypeId GetTypeId (void);

  /**
   * \brief 构造函数
   */
  SatChannel ();

  /**
   * \brief 析构函数
   */
  virtual ~SatChannel ();

  /**
   * \brief 获取信道矩阵 (重写核心方法)
   * \details 生成 3GPP 信道矩阵后，叠加物理层大尺度损耗 (FSPL + BeamGain)。
   */
  virtual Ptr<const MatrixBasedChannelModel::ChannelMatrix>
  GetChannel (Ptr<const MobilityModel> aMob,
              Ptr<const MobilityModel> bMob,
              Ptr<const PhasedArrayModel> aAntenna,
              Ptr<const PhasedArrayModel> bAntenna) override;

protected:
  /**
   * \brief 获取 3GPP 参数表 (重写参数选择逻辑)
   * \details 针对 NTN 场景，根据卫星仰角查表返回 S/Ka 频段参数。
   */
  virtual Ptr<const ParamsTable> GetThreeGppTable (Ptr<const ChannelCondition> channelCondition,
                                                   double hBS,
                                                   double hUT,
                                                   double distance2D) const override;

private:
  /**
   * \brief [核心自定义函数] 执行物理层接收功率计算
   * \return 估算的接收功率 (dBm)
   */
  double DoRxPowerCalculation (Ptr<const MobilityModel> txMobility,
                               Ptr<const MobilityModel> rxMobility,
                               Ptr<const PhasedArrayModel> txAntenna,
                               Ptr<const PhasedArrayModel> rxAntenna,
                               double txPowerDbm) const;

  /**
   * \brief 辅助函数：计算自由空间损耗 (FSPL)
   */
  double CalcFspl (double distance3D, double frequencyHz) const;

  /**
   * \brief 辅助函数：计算多波束增益/干扰 (旧接口, 假设 nadir 波束)
   */
  double CalcBeamGain (Ptr<const MobilityModel> satMob, Ptr<const MobilityModel> ueMob) const;

  // qyh 增量: 公开包装 + 多波束 boresight 偏轴 + 抛物面增益滚降
  /// 自由空间损耗 (dB), CalcFspl 公开包装
  double FsplDb (double distance3D, double frequencyHz) const;
  /// 多波束抛物面天线增益 (dBi), 偏轴角相对该波束的 boresight (波束中心)
  double BeamGainDbi (const Vector& satPos, const Vector& uePos,
                      const Vector& beamCenter, double theta3dbRad) const;
  /// 抛物面增益滚降 G(θ) = max(Gmax - 12·(θ/θ3dB)², 旁瓣地板)
  static double ParabolicGainDbi (double thetaRad, double theta3dbRad,
                                  double maxGainDbi, double sideLobeDbi);
};

} // namespace ns3

#endif /* SAT_CHANNEL_H */