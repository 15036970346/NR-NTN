/*
 * sat-phy.cc
 */

#include "sat-phy.h"
#include "ntn-mac-tags.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-channel.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/antenna-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/assert.h"
#include "ns3/node.h"
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatGeoPhy");

namespace {

// 弯管转发: 把源链路 psd 的总功率重新分布到目标链路的 SpectrumModel 上。
// 否则 SingleModelSpectrumChannel 会因 m_spectrumModel 不一致触发 NS_ASSERT。
Ptr<SpectrumValue>
RebuildPsdOnModel (Ptr<const SpectrumValue> srcPsd, Ptr<const SpectrumModel> dstModel)
{
  // 1) 累加源端总功率 (W) = sum(psd_i * BW_i)
  double totalPowerW = 0.0;
  auto bIt = srcPsd->ConstBandsBegin ();
  auto vIt = srcPsd->ConstValuesBegin ();
  while (bIt != srcPsd->ConstBandsEnd () && vIt != srcPsd->ConstValuesEnd ())
    {
      totalPowerW += (*vIt) * (bIt->fh - bIt->fl);
      ++bIt; ++vIt;
    }
  // 2) 计算目标端总带宽, 均匀铺到所有目标 band
  double totalDstBw = 0.0;
  for (auto it = dstModel->Begin (); it != dstModel->End (); ++it)
    {
      totalDstBw += (it->fh - it->fl);
    }
  Ptr<SpectrumValue> newPsd = Create<SpectrumValue> (dstModel);
  if (totalDstBw > 0)
    {
      const double psdLevel = totalPowerW / totalDstBw;
      for (size_t i = 0; i < newPsd->GetSpectrumModel ()->GetNumBands (); ++i)
        {
          (*newPsd)[i] = psdLevel;
        }
    }
  return newPsd;
}

} // anonymous namespace

// -------------------------------------------------------------------------
// SatSignalParameters Implementation
// -------------------------------------------------------------------------
SatSignalParameters::SatSignalParameters ()
{}

SatSignalParameters::SatSignalParameters (const SatSignalParameters& p)
  : SpectrumSignalParameters (p)
{
  if (p.packet)
    {
      packet = p.packet->Copy ();
    }
}

SatSignalParameters::~SatSignalParameters ()
{}

Ptr<SpectrumSignalParameters>
SatSignalParameters::Copy () const
{
  return Create<SatSignalParameters> (*this);
}


// -------------------------------------------------------------------------
// SatPhy Implementation
// -------------------------------------------------------------------------

NS_OBJECT_ENSURE_REGISTERED (SatPhy);

TypeId
SatPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatPhy")
    .SetParent<SpectrumPhy> ()
    .SetGroupName ("Spectrum")
    .AddAttribute ("TransponderGain", "Gain of the transponder in dB",
                   DoubleValue (100.0),
                   MakeDoubleAccessor (&SatPhy::SetTransponderGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NoiseFigure", "Receiver Noise Figure in dB",
                   DoubleValue (2.0),
                   MakeDoubleAccessor (&SatPhy::SetNoiseFigure),
                   MakeDoubleChecker<double> ())
    .AddTraceSource ("Tx", "Trace fired when PDU is sent.",
                     MakeTraceSourceAccessor (&SatPhy::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx", "Trace fired when PDU is received.",
                     MakeTraceSourceAccessor (&SatPhy::m_rxTrace),
                     "ns3::Packet::TracedCallback");
  return tid;
}

SatPhy::SatPhy ()
  : m_peerPhy (nullptr),
    m_mobility (nullptr),
    m_device (nullptr),
    m_temperature (290.0),
    m_processingDelay (MicroSeconds(1)),
    m_beamId (-1)
{
  m_antenna = CreateObject<IsotropicAntennaModel> ();
  m_phyTxObj = this; 
  m_phyRxObj = this;
}

SatPhy::~SatPhy () {}

void
SatPhy::DoDispose ()
{
  m_peerPhy = nullptr;
  m_channel = nullptr;
  m_phyTxObj = nullptr;
  m_phyRxObj = nullptr;
  SpectrumPhy::DoDispose ();
}

void
SatPhy::Initialize ()
{
  NS_LOG_FUNCTION (this);
  double txPowerDbm = 43.0; 
  double antennaGainDbi = 0.0; 
  double eirp = txPowerDbm + antennaGainDbi; 
  NS_LOG_INFO ("SatPhy Initialized. Base EIRP (no transponder gain): " << eirp << " dBm");
}

void
SatPhy::ConfigureRxCarriers (double carrierFreqHz, double bandwidthHz, uint32_t scs)
{
  NS_LOG_FUNCTION (this << carrierFreqHz << bandwidthHz << scs);
  m_centerFreq = carrierFreqHz;
  m_bandwidth = bandwidthHz;
  m_scs = scs;
}

Ptr<Object> SatPhy::GetPhyTx () const { return m_phyTxObj; }
Ptr<Object> SatPhy::GetPhyRx () const { return m_phyRxObj; }
void SatPhy::SetPhyTx (Ptr<Object> tx) { m_phyTxObj = tx; }
void SatPhy::SetPhyRx (Ptr<Object> rx) { m_phyRxObj = rx; }

Ptr<SpectrumChannel> SatPhy::GetTxChannel () const { return m_channel; }

void
SatPhy::SetBeamId (int beamId)
{
  NS_LOG_FUNCTION (this << beamId);
  m_beamId = beamId;
}

void
SatPhy::Receive (Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this);

  // 将通用参数转换为 SatSignalParameters 以获取 packet
  Ptr<SatSignalParameters> satParams = DynamicCast<SatSignalParameters> (params);
  if (!satParams || !satParams->packet) return;

  Ptr<Packet> p = satParams->packet;

  // PHY 收到包的时戳, 供 MAC DoPhyRx 入口算 PHY→MAC delay
  NtnPhyDeliveryTag dt (Simulator::Now ());
  p->ReplacePacketTag (dt);

  // -----------------------------------------------------------------------
  // per-packet 信道实算接收 SINR (物理基准, 供 MAC 错误模型驱动 BLER):
  //   C  = 本次收到信号 PSD 的总功率 (W) = Σ psd_i * BW_i  (含弯管增益 + NOMA β 功率切分)
  //   N  = 本端热噪声 (W) = k·T·B·NF, B 取本端 rx SpectrumModel 总带宽
  //   SINR = C / N  (单 PSD-per-packet 模型: 信道看不到同 RB 的配对叠加干扰,
  //          那部分由 NtnDciTag.effectiveSinrDb 作 NOMA 修正, 见 SatUtMac::DoPhyRx)
  // 仅在拿得到 psd 与本端 rx 频谱模型时计算, 否则不打 tag (MAC 退回错误模型自身 SINR)。
  // -----------------------------------------------------------------------
  if (satParams->psd && m_rxSpectrumModel)
    {
      double signalPowerW = 0.0;
      auto bIt = satParams->psd->ConstBandsBegin ();
      auto vIt = satParams->psd->ConstValuesBegin ();
      while (bIt != satParams->psd->ConstBandsEnd () && vIt != satParams->psd->ConstValuesEnd ())
        {
          signalPowerW += (*vIt) * (bIt->fh - bIt->fl);
          ++bIt; ++vIt;
        }

      double rxBwHz = 0.0;
      for (auto it = m_rxSpectrumModel->Begin (); it != m_rxSpectrumModel->End (); ++it)
        {
          rxBwHz += (it->fh - it->fl);
        }

      if (signalPowerW > 0.0 && rxBwHz > 0.0)
        {
          const double k = 1.380649e-23;          // Boltzmann
          const double nfLin = (m_noiseFigureLin > 0.0) ? m_noiseFigureLin : 1.0;
          const double noiseW = k * m_temperature * rxBwHz * nfLin;
          const double sinrLin = signalPowerW / (noiseW > 0.0 ? noiseW : 1e-20);
          const double sinrDb = 10.0 * std::log10 (sinrLin);
          NtnRxSinrTag rxSinrTag (sinrDb);
          p->ReplacePacketTag (rxSinrTag);
          NS_LOG_DEBUG ("SatPhy Rx per-packet channel SINR=" << sinrDb
                        << " dB (C=" << 10.0 * std::log10 (signalPowerW) + 30.0
                        << " dBm, N=" << 10.0 * std::log10 (noiseW) + 30.0 << " dBm)");
        }
    }

  m_rxTrace (p);

  bool hasError = false;
  if (hasError)
    {
      NS_LOG_WARN ("SatPhy: Packet dropped due to physical layer error.");
      return;
    }

  // 通知上层 (端点 PHY 用)
  if (!m_rxCallback.IsNull ())
    {
      m_rxCallback (p);
    }
}

void
SatPhy::SendPdu (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << p);

  if (!m_channel) return;

  m_txTrace (p);

  // 确保我们使用的是 SatSignalParameters
  Ptr<SatSignalParameters> satParams = DynamicCast<SatSignalParameters> (params);
  
  // 如果不是，创建一个新的 SatSignalParameters 并拷贝基础数据
  if (!satParams)
    {
      satParams = Create<SatSignalParameters> ();
      satParams->psd = params->psd;
      satParams->duration = params->duration;
      satParams->txPhy = params->txPhy;
      satParams->txAntenna = params->txAntenna;
      // 关键：将 params 指针更新为指向我们的子类对象，
      // 这样后续调用 StartTx 时传递的就是带有 packet 的对象
      params = satParams;
    }

  satParams->packet = p;

  params->txPhy = GetObject<SpectrumPhy> ();
  params->txAntenna = m_antenna;

  m_channel->StartTx (params);
}

void SatPhy::SetTransponderGain (double gainDb) { m_transponderGainLin = std::pow (10.0, gainDb / 10.0); }
void SatPhy::SetNoiseFigure (double nfDb) { m_noiseFigureLin = std::pow (10.0, nfDb / 10.0); }
void SatPhy::SetTemperature (double tempKelvin) { m_temperature = tempKelvin; }
void SatPhy::SetPeer (Ptr<SatPhy> peer) { m_peerPhy = peer; }

void SatPhy::StartRx (Ptr<SpectrumSignalParameters> params) { Receive(params); }
void SatPhy::SetChannel (Ptr<SpectrumChannel> channel) { m_channel = channel; }
void SatPhy::SetMobility (Ptr<MobilityModel> mobility) { m_mobility = mobility; }
void SatPhy::SetDevice (Ptr<NetDevice> device) { m_device = device; }
Ptr<MobilityModel> SatPhy::GetMobility () const { return m_mobility; }
Ptr<NetDevice> SatPhy::GetDevice () const { return m_device; }
Ptr<const SpectrumModel> SatPhy::GetRxSpectrumModel () const { return m_rxSpectrumModel; }
void SatPhy::SetRxSpectrumModel (Ptr<const SpectrumModel> model) { m_rxSpectrumModel = model; }
void SatPhy::SetRxCallback (Callback<void, Ptr<Packet>> cb) { m_rxCallback = cb; }

Ptr<Object> SatPhy::GetAntenna () const { return m_antenna; }

// -------------------------------------------------------------------------
// SatGeoUserPhy Implementation
// -------------------------------------------------------------------------

NS_OBJECT_ENSURE_REGISTERED (SatGeoUserPhy);

TypeId
SatGeoUserPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatGeoUserPhy")
    .SetParent<SatPhy> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<SatGeoUserPhy> ();
  return tid;
}

SatGeoUserPhy::SatGeoUserPhy () {}
SatGeoUserPhy::~SatGeoUserPhy () {}

void
SatGeoUserPhy::Receive (Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << "User Link Rx");

  // 防二次转发: 包的 txPhy 若已是卫星侧弯管 PHY (User/Feeder), 说明这是来自另一个
  // 共享同一 user channel 的卫星 PHY 的转发 (例如多束共信道时, 旧架构会形成回路)。
  // 这里直接丢弃, 不再二次转发。
  if (params->txPhy
      && (DynamicCast<SatGeoUserPhy> (params->txPhy)
          || DynamicCast<SatGeoFeederPhy> (params->txPhy)))
    {
      return;
    }

  SatPhy::Receive (params);

  if (!m_peerPhy) return;

  // Copy() 会调用我们自定义的 SatSignalParameters::Copy，保留 packet
  Ptr<SpectrumSignalParameters> forwardedParams = params->Copy ();

  const double k = 1.380649e-23;
  double noiseVal = k * m_temperature * m_noiseFigureLin;

  Ptr<SpectrumValue> noisePsd = Create<SpectrumValue> (forwardedParams->psd->GetSpectrumModel());
  (*noisePsd) = noiseVal;
  (*forwardedParams->psd) += (*noisePsd);

  (*forwardedParams->psd) *= m_transponderGainLin;

  CalculateSinr (forwardedParams, noiseVal);

  // 透明弯管: 转发前要把 psd 改铸到 peer (feeder) 链路的 SpectrumModel,
  // 否则 SingleModelSpectrumChannel 在 StartTx 时会断言 m_spectrumModel 不一致。
  Ptr<const SpectrumModel> peerModel = m_peerPhy->GetRxSpectrumModel ();
  if (peerModel)
    {
      forwardedParams->psd = RebuildPsdOnModel (forwardedParams->psd, peerModel);
    }

  Ptr<SatSignalParameters> satForwardParams = DynamicCast<SatSignalParameters> (forwardedParams);
  if (satForwardParams && satForwardParams->packet)
    {
      Ptr<Packet> p = satForwardParams->packet;
      Ptr<SatGeoFeederPhy> feeder = DynamicCast<SatGeoFeederPhy>(m_peerPhy);
      if (feeder)
        {
          Simulator::Schedule (m_processingDelay, &SatGeoFeederPhy::SendPduWithParams, feeder, p, forwardedParams);
        }
    }
}

void
SatGeoUserPhy::SendPduWithParams (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << "User Link Tx (Forwarding)");
  SatPhy::SendPdu (p, params);
}

double
SatGeoUserPhy::CalculateSinr (const Ptr<SpectrumSignalParameters>& params, double noisePower)
{
  double signalPower = Sum (*params->psd);
  double totalNoiseAndInterference = noisePower; 
  if (totalNoiseAndInterference <= 0) totalNoiseAndInterference = 1e-20; 
  double sinr = signalPower / totalNoiseAndInterference;
  if (sinr <= 0)
    {
      NS_FATAL_ERROR ("SatGeoUserPhy: SINR <= 0. Fatal Error.");
    }
  return sinr;
}

// -------------------------------------------------------------------------
// SatGeoFeederPhy Implementation
// -------------------------------------------------------------------------

NS_OBJECT_ENSURE_REGISTERED (SatGeoFeederPhy);

TypeId
SatGeoFeederPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatGeoFeederPhy")
    .SetParent<SatPhy> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<SatGeoFeederPhy> ();
  return tid;
}

SatGeoFeederPhy::SatGeoFeederPhy () {}
SatGeoFeederPhy::~SatGeoFeederPhy () {}

void
SatGeoFeederPhy::Receive (Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << "Feeder Link Rx");

  // 防二次转发: 共享 feeder channel 上, 任一卫星 feederPhy 发出的 UL 转发都会
  // 投递给同 channel 的其它 feederPhy。若不过滤会形成束间二次转发。
  if (params->txPhy
      && (DynamicCast<SatGeoFeederPhy> (params->txPhy)
          || DynamicCast<SatGeoUserPhy> (params->txPhy)))
    {
      return;
    }

  // BeamId 过滤: DL 包带 NtnBeamIdTag 时, 只有 beamId 匹配的本束 feederPhy 才转发,
  // 否则丢弃。不带 tag 的包走兼容路径 (单束 demo 全部转发)。
  Ptr<SatSignalParameters> satParamsForFilter = DynamicCast<SatSignalParameters> (params);
  if (satParamsForFilter && satParamsForFilter->packet)
    {
      NtnBeamIdTag bTag;
      if (satParamsForFilter->packet->PeekPacketTag (bTag)
          && m_beamId >= 0
          && static_cast<int> (bTag.GetBeamId ()) != m_beamId)
        {
          NS_LOG_DEBUG ("Feeder beam=" << m_beamId
                        << " drop DL pkt for beam=" << bTag.GetBeamId ());
          return;
        }
    }

  if (params->txPhy && params->txPhy->GetDevice() && params->txPhy->GetDevice()->GetNode())
    {
      NS_LOG_INFO ("Feeder Rx PDU from Node ID: " << params->txPhy->GetDevice()->GetNode()->GetId());
    }

  SatPhy::Receive (params);

  if (!m_peerPhy) return;

  Ptr<SpectrumSignalParameters> forwardedParams = params->Copy ();

  const double k = 1.380649e-23;
  double noiseVal = k * m_temperature * m_noiseFigureLin;
  (*forwardedParams->psd) += noiseVal;
  (*forwardedParams->psd) *= m_transponderGainLin;

  CalculateSinr (forwardedParams, noiseVal);

  // 透明弯管: 转发前把 psd 改铸到 peer (user) 链路的 SpectrumModel
  // (与 SatGeoUserPhy::Receive 对称)
  Ptr<const SpectrumModel> peerModel = m_peerPhy->GetRxSpectrumModel ();
  if (peerModel)
    {
      forwardedParams->psd = RebuildPsdOnModel (forwardedParams->psd, peerModel);
    }

  Ptr<SatSignalParameters> satForwardParams = DynamicCast<SatSignalParameters> (forwardedParams);
  if (satForwardParams && satForwardParams->packet)
    {
      Ptr<Packet> p = satForwardParams->packet;
      Ptr<SatGeoUserPhy> userPhy = DynamicCast<SatGeoUserPhy>(m_peerPhy);
      if (userPhy)
        {
          Simulator::Schedule (m_processingDelay, &SatGeoUserPhy::SendPduWithParams, userPhy, p, forwardedParams);
        }
    }
}

void
SatGeoFeederPhy::SendPduWithParams (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << "Feeder Link Tx (Forwarding)");
  NS_LOG_INFO ("Feeder Tx: Carrier Freq " << m_centerFreq << ", Duration " << params->duration);
  SatPhy::SendPdu (p, params);
}

double
SatGeoFeederPhy::CalculateSinr (const Ptr<SpectrumSignalParameters>& params, double noisePower)
{
  double signalPower = Sum (*params->psd);
  double sinr = signalPower / (noisePower + 1e-20);
  if (sinr <= 0)
    {
      NS_FATAL_ERROR ("SatGeoFeederPhy: SINR <= 0. Fatal Error.");
    }
  return sinr;
}

} // namespace ns3