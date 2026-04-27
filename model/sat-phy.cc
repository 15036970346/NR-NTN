/*
 * sat-phy.cc
 */

#include "sat-phy.h"
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

// -------------------------------------------------------------------------
// SatSignalParameters Implementation
// -------------------------------------------------------------------------
SatSignalParameters::SatSignalParameters ()
{}

SatSignalParameters::SatSignalParameters (const SatSignalParameters& p)
{
  if (p.psd)
    {
      psd = p.psd->Copy ();
    }
  duration = p.duration;
  txPhy = p.txPhy;
  txAntenna = p.txAntenna;

  if (p.packet)
    {
      packet = p.packet->Copy ();
    }
}

SatSignalParameters::~SatSignalParameters ()
{}

Ptr<SpectrumSignalParameters>
SatSignalParameters::Copy ()
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
void SatPhy::SetRxPacketCallback (Callback<void, Ptr<Packet>> callback) { m_rxPacketCallback = callback; }

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

  m_rxTrace (p);

  if (!m_rxPacketCallback.IsNull ())
    {
      m_rxPacketCallback (p->Copy ());
    }

  bool hasError = false; 
  if (hasError)
    {
      NS_LOG_WARN ("SatPhy: Packet dropped due to physical layer error.");
      return; 
    }
}

void
SatPhy::SendPdu (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this << p);

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

  if (!m_channel)
    {
      if (m_peerPhy)
        {
          NS_LOG_INFO ("SatPhy: no SpectrumChannel, using direct peer delivery fallback.");
          Simulator::Schedule (m_processingDelay,
                               &SatPhy::StartRx,
                               m_peerPhy,
                               satParams->Copy ());
        }
      return;
    }

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
Ptr<const SpectrumModel> SatPhy::GetRxSpectrumModel () const { return nullptr; } 

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

  SatPhy::Receive (params); 
  
  if (!m_peerPhy) return; 

  // Copy() 会调用我们自定义的 SatSignalParameters::Copy，保留 packet
  Ptr<SpectrumSignalParameters> forwardedParams = params->Copy ();
  if (!forwardedParams->psd)
    {
      NS_LOG_WARN ("SatGeoUserPhy: PSD unavailable, bypassing SINR/noise processing for packet forwarding.");

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
      return;
    }
  
  const double k = 1.380649e-23;
  double noiseVal = k * m_temperature * m_noiseFigureLin; 
  
  Ptr<SpectrumValue> noisePsd = Create<SpectrumValue> (forwardedParams->psd->GetSpectrumModel());
  (*noisePsd) = noiseVal;
  (*forwardedParams->psd) += (*noisePsd);

  (*forwardedParams->psd) *= m_transponderGainLin;

  CalculateSinr (forwardedParams, noiseVal);

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
  
  if (params->txPhy && params->txPhy->GetDevice() && params->txPhy->GetDevice()->GetNode())
    {
      NS_LOG_INFO ("Feeder Rx PDU from Node ID: " << params->txPhy->GetDevice()->GetNode()->GetId());
    }

  SatPhy::Receive (params); 

  if (!m_peerPhy) return;

  Ptr<SpectrumSignalParameters> forwardedParams = params->Copy ();
  if (!forwardedParams->psd)
    {
      NS_LOG_WARN ("SatGeoFeederPhy: PSD unavailable, bypassing SINR/noise processing for packet forwarding.");

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
      return;
    }
  
  const double k = 1.380649e-23;
  double noiseVal = k * m_temperature * m_noiseFigureLin;
  (*forwardedParams->psd) += noiseVal;
  (*forwardedParams->psd) *= m_transponderGainLin;

  CalculateSinr (forwardedParams, noiseVal);

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
