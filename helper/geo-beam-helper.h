#ifndef GEO_BEAM_HELPER_H
#define GEO_BEAM_HELPER_H

#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"
#include "ns3/spectrum-channel.h"
#include "ns3/spectrum-model.h"
#include "ns3/vector.h"
#include "ns3/epc-helper.h"
#include "ns3/nr-helper.h"
#include "../model/sat-phy.h"
#include "../model/frequency-reuse.h"
#include <vector>
#include <map>

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

  /// 设置馈电链路中心频率/带宽 (默认 30 GHz / 100 MHz)
  void SetFeederConf (double centerFreqHz, double bandwidthHz);

  /// 实际接线的波束数 (默认 1)。每束有独立 user channel/SpectrumModel,
  /// SatGeoFeederPhy 用 NtnBeamIdTag 在共享 feeder channel 上做 beam 路由。
  void SetActiveBeamCount (uint32_t n);

  /**
   * \brief 获取当前配置的波束频点信息
   */
  std::vector<BeamFrequencyConfig> GetBeamFrequencyConfig () const;

  /// 用户链路信道 (兼容旧 API: 返回第一个波束的 user channel)
  Ptr<SpectrumChannel> GetSpectrumChannel (void) const;
  /// 馈电链路信道 (GW 与卫星 SatGeoFeederPhy 共用)
  Ptr<SpectrumChannel> GetFeederChannel (void) const;
  /// 兼容旧 API: 返回第一个波束的 user SpectrumModel
  Ptr<const SpectrumModel> GetUserSpectrumModel (void) const;
  Ptr<const SpectrumModel> GetFeederSpectrumModel (void) const;

  /// 多波束扩展: 按 beamId 取出该束的独立 user channel / SpectrumModel
  Ptr<SpectrumChannel>     GetUserChannel        (uint32_t beamId) const;
  Ptr<const SpectrumModel> GetUserSpectrumModel  (uint32_t beamId) const;

  /// 按波束 id 取出对应卫星 PHY (Install 之后可用)
  Ptr<SatGeoUserPhy>   GetUserPhy   (uint32_t beamId) const;
  Ptr<SatGeoFeederPhy> GetFeederPhy (uint32_t beamId) const;

  // ===== 频率复用 + 同色波束 C/I 模型 =====
  /// 当前 reuse 配置 (Install 后会按 m_beamFreqConfig 的 colorId 注入每束)
  Ptr<FrequencyReuse> GetFrequencyReuse () const;
  /// 单束主瓣峰值天线增益 (dBi), 默认 43
  void   SetMainLobeGainDbi (double v);
  double GetMainLobeGainDbi () const;
  /// 同色波束方向上的旁瓣增益 (dBi), 默认 25
  void   SetSideLobeGainDbi (double v);
  double GetSideLobeGainDbi () const;
  /// 单干扰束 C/I_co = MainLobe - SideLobe (dB), 默认 18
  double GetCoBeamCIDb () const;

private:
  // 为卫星安装多个点波束: 每束创建一对 SatGeoFeederPhy + SatGeoUserPhy 并 SetPeer。
  // 返回该卫星节点上挂的 NetDevice 列表(目前仍创建 NrGnbNetDevice 占位,以保持原 API)。
  NetDeviceContainer InstallBeams (Ptr<Node> satNode);

  // 配置卫星交换机：将波束接口和馈电接口桥接
  Ptr<NetDevice> InstallSwitch (Ptr<Node> satNode, NetDeviceContainer beamDevices, Ptr<NetDevice> feederDevice);

  Ptr<const SpectrumModel> BuildBeamUserSpectrumModel (const BeamFrequencyConfig& cfg) const;
  Ptr<const SpectrumModel> BuildFeederSpectrumModel () const;

  ObjectFactory m_deviceFactory;
  ObjectFactory m_phyFactory;
  ObjectFactory m_channelFactory;
  ObjectFactory m_antennaFactory;

  /// 每波束独立 user channel + SpectrumModel (按 beamId 索引)
  std::map<uint32_t, Ptr<SpectrumChannel>>     m_userChannels;
  std::map<uint32_t, Ptr<const SpectrumModel>> m_userSpectrumModels;
  /// 共享馈电链路 channel + SpectrumModel
  Ptr<SpectrumChannel>     m_feederChannel;
  Ptr<const SpectrumModel> m_feederSpectrumModel;

  double m_centerFreqHz;
  double m_bandwidthHz;
  double m_feederCenterFreqHz;
  double m_feederBandwidthHz;
  uint32_t m_activeBeamCount;

  FrequencyReuseMode m_reuseMode;  //!< 频率复用模式
  std::vector<BeamFrequencyConfig> m_beamFreqConfig;  //!< 各波束频点配置

  /// 同色 C/I 解析模型用 (helper 注入到 SatUtPhy)
  Ptr<FrequencyReuse> m_freqReuse;
  double m_mainLobeGainDbi {43.0};
  double m_sideLobeGainDbi {25.0};

  // 安装后按 beamId 索引到对应的卫星 PHY 对, 便于上层访问
  std::map<uint32_t, Ptr<SatGeoUserPhy>>   m_userPhys;
  std::map<uint32_t, Ptr<SatGeoFeederPhy>> m_feederPhys;
};

} // namespace ns3

#endif /* GEO_BEAM_HELPER_H */
