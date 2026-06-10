/*
 * sat-phy.h
 * Fixed: Added missing header for SpectrumSignalParameters
 */

#ifndef SAT_GEO_PHY_H
#define SAT_GEO_PHY_H

#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-value.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "ns3/event-id.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/net-device.h"
// [Fix] 必须包含这个文件，否则编译器找不到基类的定义
#include "ns3/spectrum-signal-parameters.h"

namespace ns3 {

class SpectrumChannel;
class AntennaModel;

// 自定义信号参数结构体，用于携带 Packet
struct SatSignalParameters : public SpectrumSignalParameters
{
  SatSignalParameters ();
  SatSignalParameters (const SatSignalParameters& p);
  virtual ~SatSignalParameters ();

  // 必须与基类 SpectrumSignalParameters::Copy() const 同签名才能真正 override,
  // 否则 SpectrumChannel 复制信号时会退化成基类, packet 字段会丢失。
  Ptr<SpectrumSignalParameters> Copy () const override;

  Ptr<Packet> packet;
};

/**
 * \brief SatPhy: The base class for satellite physical layer.
 */
class SatPhy : public SpectrumPhy
{
public:
  static TypeId GetTypeId (void);
  SatPhy ();
  virtual ~SatPhy ();

  virtual void Initialize ();
  void ConfigureRxCarriers (double carrierFreqHz, double bandwidthHz, uint32_t scs);

  Ptr<Object> GetPhyTx () const;
  Ptr<Object> GetPhyRx () const;
  void SetPhyTx (Ptr<Object> tx);
  void SetPhyRx (Ptr<Object> rx);

  Ptr<SpectrumChannel> GetTxChannel () const;

  virtual void SendPdu (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params);
  void SetBeamId (int beamId);
  virtual void Receive (Ptr<SpectrumSignalParameters> params);

  // --- NS-3 SpectrumPhy Standard Interfaces ---
  virtual void SetChannel (Ptr<SpectrumChannel> channel) override;
  virtual void SetMobility (Ptr<MobilityModel> mobility) override;
  virtual void SetDevice (Ptr<NetDevice> device) override;
  virtual Ptr<MobilityModel> GetMobility () const override;
  virtual Ptr<NetDevice> GetDevice () const override;
  virtual Ptr<const SpectrumModel> GetRxSpectrumModel () const override;
  
  virtual Ptr<Object> GetAntenna () const override;
  virtual void StartRx (Ptr<SpectrumSignalParameters> params) override;

  // --- Configuration Helpers ---
  void SetTransponderGain (double gainDb);
  void SetNoiseFigure (double nfDb);
  void SetTemperature (double tempKelvin);
  void SetPeer (Ptr<SatPhy> peer);

  /// 显式设置 Rx SpectrumModel (SpectrumChannel 投递包前用此识别)
  void SetRxSpectrumModel (Ptr<const SpectrumModel> model);

  /// 上层接收回调: 收到本端 Rx 时 (在 Rx trace 之后) 触发, 用于把 packet 交给 MAC 等上层。
  /// 注意: 卫星上的 SatGeoUserPhy/SatGeoFeederPhy 完全 override Receive 做透明转发,
  /// 不会调到这个回调; 这是给端点 PHY (NtnGwPhy / 给 UE 用的 SatPhy) 用的。
  void SetRxCallback (Callback<void, Ptr<Packet>> cb);
  /// qyh 接口别名 (SetRxCallback 的同义): 兼容 qyh demo
  void SetRxPacketCallback (Callback<void, Ptr<Packet>> cb) { SetRxCallback (cb); }

protected:
  virtual void DoDispose (void) override;

  Ptr<SatPhy> m_peerPhy;
  Ptr<SpectrumChannel> m_channel;
  Ptr<MobilityModel> m_mobility;
  Ptr<NetDevice> m_device;
  Ptr<AntennaModel> m_antenna;

  double m_transponderGainLin;
  double m_noiseFigureLin;
  double m_temperature;
  Time m_processingDelay;
  
  double m_centerFreq;
  double m_bandwidth;
  uint32_t m_scs;
  int m_beamId;

  Ptr<Object> m_phyTxObj;
  Ptr<Object> m_phyRxObj;
  Ptr<const SpectrumModel> m_rxSpectrumModel;
  Callback<void, Ptr<Packet>> m_rxCallback;

  TracedCallback<Ptr<const Packet>> m_txTrace;
  TracedCallback<Ptr<const Packet>> m_rxTrace;
};

class SatGeoUserPhy : public SatPhy
{
public:
  static TypeId GetTypeId (void);
  SatGeoUserPhy ();
  virtual ~SatGeoUserPhy ();

  void SendPduWithParams (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params);
  virtual void Receive (Ptr<SpectrumSignalParameters> params) override;
  double CalculateSinr (const Ptr<SpectrumSignalParameters>& params, double noisePower);
};

class SatGeoFeederPhy : public SatPhy
{
public:
  static TypeId GetTypeId (void);
  SatGeoFeederPhy ();
  virtual ~SatGeoFeederPhy ();

  void SendPduWithParams (Ptr<Packet> p, Ptr<SpectrumSignalParameters> params);
  virtual void Receive (Ptr<SpectrumSignalParameters> params) override;
  double CalculateSinr (const Ptr<SpectrumSignalParameters>& params, double noisePower);
};

} // namespace ns3

#endif /* SAT_GEO_PHY_H */