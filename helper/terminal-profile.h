#ifndef SAT_TERMINAL_PROFILE_H
#define SAT_TERMINAL_PROFILE_H

#include "ns3/object.h"
#include "../model/resource-manager.h"

#include <string>

namespace ns3 {

enum SatDataServiceType {
  SAT_DATA_HTTP,
  SAT_DATA_FTP
};

/**
 * \brief Lightweight per-UE profile attached directly to each node.
 *
 * This makes terminal classification explicit and queryable from any scenario
 * code instead of re-deriving it from node indices.
 */
class SatTerminalProfile : public Object
{
public:
  static TypeId GetTypeId (void);

  SatTerminalProfile ();
  ~SatTerminalProfile () override;

  void SetTerminalType (UtType type);
  UtType GetTerminalType () const;

  void SetBeamId (uint16_t beamId);
  uint16_t GetBeamId () const;

  void SetVoiceEnabled (bool enabled);
  bool HasVoiceService () const;

  void SetDataServiceType (SatDataServiceType type);
  SatDataServiceType GetDataServiceType () const;

  std::string DescribeTrafficProfile () const;

private:
  UtType m_terminalType;
  uint16_t m_beamId;
  bool m_voiceEnabled;
  SatDataServiceType m_dataServiceType;
};

} // namespace ns3

#endif /* SAT_TERMINAL_PROFILE_H */
