/*
 * 文件: contrib/geo-sat/model/ntn-phy-error-model.cc
 */
#include "ntn-phy-error-model.h"
#include "ns3/log.h"
#include "ns3/double.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnPhyErrorModel");
NS_OBJECT_ENSURE_REGISTERED (NtnPhyErrorModel);

TypeId
NtnPhyErrorModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnPhyErrorModel")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnPhyErrorModel> ()
    .AddAttribute ("SinrDb",
                   "静态 SINR (dB), 在 SetSinrProvider 未调用时使用",
                   DoubleValue (20.0),
                   MakeDoubleAccessor (&NtnPhyErrorModel::m_sinrDb),
                   MakeDoubleChecker<double> ());
  return tid;
}

NtnPhyErrorModel::NtnPhyErrorModel ()
{
  m_rng = CreateObject<UniformRandomVariable> ();
  m_rng->SetAttribute ("Min", DoubleValue (0.0));
  m_rng->SetAttribute ("Max", DoubleValue (1.0));
}

NtnPhyErrorModel::~NtnPhyErrorModel () = default;

double
NtnPhyErrorModel::BlerFromSinr (double s)
{
  if (s <  -5.0) return 1.0;
  if (s <   0.0) return 1.0 + (0.5 - 1.0)  * (s - (-5.0)) / 5.0;     // 1.0 -> 0.5
  if (s <   5.0) return 0.5 + (0.1 - 0.5)  * (s -   0.0)  / 5.0;     // 0.5 -> 0.1
  if (s <  10.0) return 0.1 + (0.01 - 0.1) * (s -   5.0)  / 5.0;     // 0.1 -> 0.01
  return 0.001;
}

void   NtnPhyErrorModel::SetSinrDb       (double s) { m_sinrDb = s; m_useProvider = false; m_sinrExplicitlySet = true; }
double NtnPhyErrorModel::GetSinrDb       () const
{
  return m_useProvider && !m_sinrProvider.IsNull () ? m_sinrProvider () : m_sinrDb;
}
void   NtnPhyErrorModel::SetSinrProvider (Callback<double> p) { m_sinrProvider = p; m_useProvider = true; }

double
NtnPhyErrorModel::GetBler () const
{
  return BlerFromSinr (GetSinrDb ());
}

bool
NtnPhyErrorModel::ShouldDrop ()
{
  m_checkCount++;
  double bler = GetBler ();
  if (bler <= 0.0) return false;
  if (bler >= 1.0)
    {
      m_dropCount++;
      return true;
    }
  bool drop = (m_rng->GetValue () < bler);
  if (drop) m_dropCount++;
  return drop;
}

} // namespace ns3
