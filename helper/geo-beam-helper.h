#ifndef GEO_BEAM_HELPER_H
#define GEO_BEAM_HELPER_H

#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"
#include "ns3/spectrum-channel.h"
#include "ns3/vector.h"
#include "ns3/epc-helper.h"
#include "ns3/nr-helper.h"
#include <vector>

namespace ns3 {

/**
 * \brief 频率复用模式枚举
 */
enum class FrequencyReuseMode
{
  FR1_4COLOR = 4,  //!< 4色复用模式
  FR1_7COLOR = 7   //!< 7色复用模式
};

/**
 * \brief 波束频点配置结构
 */
struct BeamFrequencyConfig
{
  uint16_t beamId;        //!< 波束ID (1-7)
  double centerFreqHz;     //!< 波束中心频率 (Hz)
  double bandwidthHz;      //!< 波束带宽 (Hz)
  uint32_t earfcn;         //!< EARFCN 频点号
  uint8_t colorId;        //!< 颜色ID (复用区分)
};

extern const std::vector<BeamFrequencyConfig> g_4ColorFrequencyConfig; //!< 4色复用频点配置
extern const std::vector<BeamFrequencyConfig> g_7ColorFrequencyConfig; //!< 7色复用频点配置

/**
 * \brief Geo卫星节点助手
 * \details 负责：
 * 1. 创建 gNB 节点（卫星）。
 * 2. 安装 SatSGP4MobilityModel。
 * 3. 安装 SatPhy 和 SpectrumChannel，创建 SatNetDevice。
 * 4. 配置 4色/7色 频率复用波束。
 * 5. 配置卫星交换机 (Bridge)，实现波束间/馈电链路的数据交换。
 */
class GeoBeamHelper
{
public:
  GeoBeamHelper ();
  virtual ~GeoBeamHelper ();

  void SetPhy (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetDevice (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetChannel (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());

  /**
   * \brief 设置频率复用模式
   * \param mode 复用模式 (4: 4色复用, 7: 7色复用)
   * \details
   * - 统一口径: 7个波束，每波束5MHz，总名义带宽35MHz
   * - 4色复用: 5MHz每色组，部分波束复用同一载频
   * - 7色复用: 5MHz每波束，7个波束使用7个独立频点
   */
  void SetFrequencyReuseMode (uint8_t mode);

  /**
   * \brief 安装卫星节点的核心逻辑
   * \param nodes 卫星节点容器
   * \param tleLines TLE 轨道数据
   * \param feederChannel (可选) 馈电链路信道，用于连接 GW
   * \return 安装的所有 NetDevice（包括 Bridge 和各个波束接口）
   */
  NetDeviceContainer Install (NodeContainer nodes, std::vector<std::string> tleLines = {});

  void SetBeamConf (double centerFreqHz, double bandwidthHz);

  /**
   * \brief 获取当前配置的波束频点信息
   */
  std::vector<BeamFrequencyConfig> GetBeamFrequencyConfig () const;

  Ptr<SpectrumChannel> GetSpectrumChannel (void) const;

private:
  // 为卫星安装多个点波束 (User Links)
  NetDeviceContainer InstallBeams (Ptr<Node> satNode, Ptr<SpectrumChannel> channel);

  // 配置卫星交换机：将波束接口和馈电接口桥接
  Ptr<NetDevice> InstallSwitch (Ptr<Node> satNode, NetDeviceContainer beamDevices, Ptr<NetDevice> feederDevice);

  ObjectFactory m_deviceFactory;
  ObjectFactory m_phyFactory;
  ObjectFactory m_channelFactory;
  ObjectFactory m_antennaFactory;

  Ptr<SpectrumChannel> m_channel;
  double m_centerFreqHz;
  double m_bandwidthHz;

  FrequencyReuseMode m_reuseMode;  //!< 频率复用模式
  std::vector<BeamFrequencyConfig> m_beamFreqConfig;  //!< 各波束频点配置
};

} // namespace ns3

#endif /* GEO_BEAM_HELPER_H */
