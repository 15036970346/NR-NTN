/*
 * ntn-config-helper.h
 * NTN (Non-Terrestrial Network) configuration helper
 * Handles RLC/PDCP parameters optimized for GEO satellite latency
 */

#ifndef NTN_CONFIG_HELPER_H
#define NTN_CONFIG_HELPER_H

#include "ns3/core-module.h"
#include "ns3/ptr.h"

namespace ns3 {

class RohcCompressor;
class SatPdcp;
class HarqManager;

class NtnConfigHelper
{
public:
  NtnConfigHelper ();
  ~NtnConfigHelper ();

  static void ConfigureRlcForGeoDelay ();
  static void ConfigurePdcpForGeoDelay ();
  static void ConfigureAll ();
  static void SetGeoRoundTripTime (Time rtt);
  static void SetpollRetransmitTimer (Time timer);
  static void SetreorderingTimer (Time timer);
  static void SetstatusProhibitTimer (Time timer);

  static void EnableRohc (bool enable);
  static void SetRohcMode (uint8_t mode);
  static Ptr<RohcCompressor> CreateRohcCompressor (uint16_t cid);
  static Ptr<SatPdcp> CreatePdcpWithRohc (uint16_t rnti, uint8_t lcId);

  /**
   * \brief 配置HARQ管理器
   * \param harqManager HARQ管理器实例
   * \param enableHarq true: 启用HARQ重传; false: 禁用HARQ（NACK不触发重传）
   * \details 用于对比HARQ开启/关闭时的系统性能差异
   */
  static void ConfigureHarqManager (Ptr<HarqManager> harqManager, bool enableHarq);

private:
  static Time m_pollRetransmitTimer;
  static Time m_reorderingTimer;
  static Time m_statusProhibitTimer;
  static bool m_rohcEnabled;
  static uint8_t m_rohcMode;
};

} // namespace ns3

#endif
