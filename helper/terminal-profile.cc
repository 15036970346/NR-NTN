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
  : m_terminalType (UT_CONSUMER),//默认终端类型是消费级终端
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
SatTerminalProfile::SetVoiceEnabled (bool enabled)//设置是否开启语音业务
{
  m_voiceEnabled = enabled;
}

bool//查询是否有语音业务
SatTerminalProfile::HasVoiceService () const
{
  return m_voiceEnabled;
}

void
SatTerminalProfile::SetDataServiceType (SatDataServiceType type)//设置数据业务类型
{
  m_dataServiceType = type;
}

SatDataServiceType//获取数据业务类型
SatTerminalProfile::GetDataServiceType () const
{
  return m_dataServiceType;
}

std::string//生成终端业务画像描述字符串
SatTerminalProfile::DescribeTrafficProfile () const
{
  std::string terminal = (m_terminalType == UT_CONSUMER) ? "UT_CONSUMER" : "UT_PORTABLE";
  std::string data = (m_dataServiceType == SAT_DATA_HTTP) ? "HTTP" : "FTP";
  std::string voice = m_voiceEnabled ? "VoIP" : "NoVoice";
  return terminal + "/" + voice + "+" + data;
}

} // namespace ns3
