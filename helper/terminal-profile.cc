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
    m_beamId (0)
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

} // namespace ns3
