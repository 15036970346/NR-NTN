/*
 * ntn-config-helper.cc
 * NTN configuration helper implementation
 * Optimizes RLC/PDCP parameters for GEO satellite ~600ms+ RTT
 */

#include "ntn-config-helper.h"
#include "ns3/log.h"
#include "../model/rohc-compressor.h"
#include "../model/sat-pdcp.h"
#include "../model/harq-manager.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnConfigHelper");

Time NtnConfigHelper::m_pollRetransmitTimer = MilliSeconds (650);
Time NtnConfigHelper::m_reorderingTimer = MilliSeconds (650);
Time NtnConfigHelper::m_statusProhibitTimer = MilliSeconds (100);
bool NtnConfigHelper::m_rohcEnabled = true;
uint8_t NtnConfigHelper::m_rohcMode = 1;

NtnConfigHelper::NtnConfigHelper ()
{
  NS_LOG_FUNCTION (this);
}

NtnConfigHelper::~NtnConfigHelper ()
{
  NS_LOG_FUNCTION (this);
}

void
NtnConfigHelper::SetpollRetransmitTimer (Time timer)
{
  m_pollRetransmitTimer = timer;
}

void
NtnConfigHelper::SetreorderingTimer (Time timer)
{
  m_reorderingTimer = timer;
}

void
NtnConfigHelper::SetstatusProhibitTimer (Time timer)
{
  m_statusProhibitTimer = timer;
}

void
NtnConfigHelper::SetGeoRoundTripTime (Time rtt)
{
  NS_LOG_FUNCTION (rtt.GetMilliSeconds () << "ms");
  m_pollRetransmitTimer = rtt + MilliSeconds (50);
  m_reorderingTimer = rtt + MilliSeconds (50);
}

void
NtnConfigHelper::EnableRohc (bool enable)
{
  m_rohcEnabled = enable;
}

void
NtnConfigHelper::SetRohcMode (uint8_t mode)
{
  m_rohcMode = mode;
}

Ptr<RohcCompressor>
NtnConfigHelper::CreateRohcCompressor (uint16_t cid)
{
  Ptr<RohcCompressor> rohc = CreateObject<RohcCompressor> ();
  rohc->SetCid (cid);
  rohc->SetMode (static_cast<RohcMode> (m_rohcMode));
  rohc->SetAttribute ("Enabled", BooleanValue (m_rohcEnabled));
  NS_LOG_INFO ("Created ROHC Compressor: CID=" << cid << ", Mode=" << (uint32_t)m_rohcMode);
  return rohc;
}

Ptr<SatPdcp>
NtnConfigHelper::CreatePdcpWithRohc (uint16_t rnti, uint8_t lcId)
{
  NS_LOG_FUNCTION (rnti << (uint32_t)lcId);
  
  Ptr<SatPdcp> pdcp = CreateObject<SatPdcp> ();
  pdcp->SetRnti (rnti);
  pdcp->SetLcId (lcId);
  
  Ptr<RohcCompressor> rohc = CreateRohcCompressor (rnti);
  pdcp->SetRohcCompressor (rohc);
  
  NS_LOG_INFO ("Created SatPdcp with ROHC: RNTI=" << rnti 
               << ", LCID=" << (uint32_t)lcId
               << ", ROHC=" << (rohc->IsEnabled() ? "Enabled" : "Disabled"));
  
  return pdcp;
}

void
NtnConfigHelper::ConfigureRlcForGeoDelay ()
{
  NS_LOG_FUNCTION ("" << m_pollRetransmitTimer.GetMilliSeconds ());
  NS_LOG_INFO ("Configuring RLC for GEO delay: "
               << "PollRetransmit=" << m_pollRetransmitTimer.GetMilliSeconds () << "ms, "
               << "Reordering=" << m_reorderingTimer.GetMilliSeconds () << "ms, "
               << "StatusProhibit=" << m_statusProhibitTimer.GetMilliSeconds () << "ms");

  Config::SetDefault ("ns3::LteRlcAm::PollRetransmitTimer", TimeValue (m_pollRetransmitTimer));
  Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (m_reorderingTimer));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (m_statusProhibitTimer));
  
  NS_LOG_INFO ("RLC GEO adaptation complete: increased timers to >600ms RTT");
}

void
NtnConfigHelper::ConfigurePdcpForGeoDelay ()
{
  NS_LOG_FUNCTION ("");
  NS_LOG_INFO ("Configuring PDCP for GEO delay with ROHC: Enabled=" << m_rohcEnabled 
               << ", Mode=" << (uint32_t)m_rohcMode);
}

void
NtnConfigHelper::ConfigureAll ()
{
  NS_LOG_FUNCTION ("");
  NS_LOG_INFO ("--- Configuring NTN RLC/PDCP for GEO Satellite ---");
  ConfigureRlcForGeoDelay ();
  ConfigurePdcpForGeoDelay ();
  NS_LOG_INFO ("--- NTN Configuration Complete ---");
}

void
NtnConfigHelper::ConfigureHarqManager (Ptr<HarqManager> harqManager, bool enableHarq)
{
  NS_LOG_FUNCTION (enableHarq);

  if (harqManager == nullptr)
    {
      NS_LOG_ERROR ("[NTN-Config] HarqManager is null, cannot configure!");
      return;
    }

  harqManager->SetHarqEnabled (enableHarq);

  if (enableHarq)
    {
      NS_LOG_INFO ("[NTN-Config] HARQ Enabled - NACK will trigger IR retransmission");
      NS_LOG_INFO ("[NTN-Config]   - Max retransmissions: 4");
      NS_LOG_INFO ("[NTN-Config]   - IR RV sequence: 0 -> 2 -> 3 -> 1");
      NS_LOG_INFO ("[NTN-Config]   - Expected gain: ~3 dB");
    }
  else
    {
      NS_LOG_INFO ("[NTN-Config] HARQ Disabled - NACK will be treated as success (packet dropped)");
      NS_LOG_INFO ("[NTN-Config]   - This mode is used for baseline comparison");
    }
}

} // namespace ns3
