#include "terminal-profile.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (SatTerminalProfile);

TypeId
SatTerminalProfile::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatTerminalProfile")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatTerminalProfile> ();
  return tid;
}

SatTerminalProfile::SatTerminalProfile ()
  : m_terminalType (UT_CONSUMER),
    m_beamId (0),
    m_voiceEnabled (true),
    m_dataServiceType (SAT_DATA_HTTP)
{
}

SatTerminalProfile::~SatTerminalProfile () = default;

void
SatTerminalProfile::SetTerminalType (UtType type)
{
  m_terminalType = type;
}

UtType
SatTerminalProfile::GetTerminalType () const
{
  return m_terminalType;
}

void
SatTerminalProfile::SetBeamId (uint16_t beamId)
{
  m_beamId = beamId;
}

uint16_t
SatTerminalProfile::GetBeamId () const
{
  return m_beamId;
}

void
SatTerminalProfile::SetVoiceEnabled (bool enabled)
{
  m_voiceEnabled = enabled;
}

bool
SatTerminalProfile::HasVoiceService () const
{
  return m_voiceEnabled;
}

void
SatTerminalProfile::SetDataServiceType (SatDataServiceType type)
{
  m_dataServiceType = type;
}

SatDataServiceType
SatTerminalProfile::GetDataServiceType () const
{
  return m_dataServiceType;
}

std::string
SatTerminalProfile::DescribeTrafficProfile () const
{
  std::string terminal = (m_terminalType == UT_CONSUMER) ? "UT_CONSUMER" : "UT_PORTABLE";
  std::string data = (m_dataServiceType == SAT_DATA_HTTP) ? "HTTP" : "FTP";
  std::string voice = m_voiceEnabled ? "VoIP" : "NoVoice";
  return terminal + "/" + voice + "+" + data;
}

} // namespace ns3
