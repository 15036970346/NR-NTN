/*
 * 文件路径: contrib/geo-sat/model/ntn-gw-phy.cc
 * 实现: 见 ntn-gw-phy.h 顶部说明。NtnGwPhy 不引入新行为, 只是 SatPhy 的语义标识子类。
 */
#include "ntn-gw-phy.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnGwPhy");
NS_OBJECT_ENSURE_REGISTERED (NtnGwPhy);

TypeId
NtnGwPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnGwPhy")
    .SetParent<SatPhy> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnGwPhy> ();
  return tid;
}

NtnGwPhy::NtnGwPhy ()
{
  NS_LOG_FUNCTION (this);
}

NtnGwPhy::~NtnGwPhy ()
{
  NS_LOG_FUNCTION (this);
}

} // namespace ns3
