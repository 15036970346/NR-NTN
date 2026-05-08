/*
 * traffic-models.h
 *
 * GEO traffic helper that keeps the original geo-sat API surface while
 * delegating HTTP / FTP / VoIP generation to reusable ns-3 applications.
 */

#ifndef SAT_TRAFFIC_MODELS_H
#define SAT_TRAFFIC_MODELS_H

#include "ns3/address.h"
#include "ns3/application-container.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/socket.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "ns3/type-id.h"

#include <iosfwd>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace ns3 {

class Node;
class PacketSink;
class UniformRandomVariable;
class ThreeGppHttpServer;
class ThreeGppHttpVariables;

enum TrafficModelType {
    TRAFFIC_FULL_BUFFER,
    TRAFFIC_FTP,
    TRAFFIC_HTTP,
    TRAFFIC_VOIP_RTP,
    TRAFFIC_LEGACY_DATA,
    TRAFFIC_LEGACY_VOICE,
};

struct TrafficModelStats {
    uint64_t totalBytesGenerated {0};
    uint64_t totalPacketsGenerated {0};
    uint64_t totalBytesSent {0};
    uint64_t usefulBytesDelivered {0};
    uint64_t sendBlockedCount {0};
    Time lastGenerateTime {Time (0)};
    double avgGenerateIntervalUs {0};
    double peakThroughputBps {0};
    double beamPeakThroughputBps {0};
    uint32_t ipv4Packets {0};
    uint32_t ipv6Packets {0};
    std::map<uint32_t, uint64_t> perUserUsefulBytes;
    std::map<uint32_t, double> perUserPeakBps;
};

struct InstalledTerminalTraffic {
    bool isConsumerPhone {false};
    bool hasVoip {false};
    bool hasHttp {false};
    bool hasFtp {false};
};

class SatTrafficGenerator : public Object
{
  public:
    static TypeId GetTypeId (void);

    SatTrafficGenerator ();
    ~SatTrafficGenerator () override;

    void SetTrafficModel (TrafficModelType type);
    TrafficModelType GetTrafficModel () const;
    std::vector<TrafficModelType> GetInstalledTrafficModels () const;
    std::string DescribeInstalledTrafficModels () const;

    void SetTargetRate (uint64_t rateBitsPerSec);
    void SetGeoRtt (Time rtt);
    void SetMaxPrbPerTti (uint32_t numPrb);
    void SetRohcOverheadBytes (uint32_t bytes);
    void SetApplicationWindow (Time start, Time stop);
    void EnableClosedLoopControl (bool enable, uint8_t lowWatermarkPercent = 80);
    void SetTcpCongestionTypeId (TypeId typeId);
    TypeId GetTcpCongestionTypeId () const;

    ApplicationContainer InstallFullBuffer (NodeContainer sourceNodes,
                                            Ipv4Address sinkAddress,
                                            bool isUplink = false);
    ApplicationContainer InstallFtp (NodeContainer sourceNodes,
                                     Ipv4Address sinkAddress,
                                     bool isUplink = false);
    ApplicationContainer InstallHttp (NodeContainer sourceNodes,
                                      Ipv4Address sinkAddress,
                                      bool isUplink = false);
    ApplicationContainer InstallHttpDownload (NodeContainer contentServerNodes,
                                              Ipv4Address clientAddress);
    ApplicationContainer InstallVoipRtp (NodeContainer sourceNodes,
                                         Ipv4Address sinkAddress,
                                         bool isUplink = false);
    ApplicationContainer InstallLegacyConsumerData (NodeContainer sourceNodes,
                                                    Ipv4Address sinkAddress,
                                                    bool isUplink = false);

    InstalledTerminalTraffic InstallProfileTraffic (NodeContainer sourceNodes,
                                                    Ptr<Node> terminalNode,
                                                    Ipv4Address terminalAddress,
                                                    bool isUplink = false);

    TrafficModelStats GetStats () const { return m_totalStats; }
    TrafficModelStats GetStats (TrafficModelType type) const;
    void ResetStats ();
    void DumpStats (std::ostream& os) const;

    // Provide queue-utilization feedback from an external scheduler/MAC module.
    void ReportQueueUtilization (double utilizationPercent);
    void SetQueueStateCallback (Callback<void, double> cb);
    void SetTxTraceCallback (Callback<void, Ptr<const Packet>, const Address&> cb);
    void OnTypedPacketTx (TrafficModelType type, uint32_t nodeId, Ptr<const Packet> packet);

    ApplicationContainer GenerateVoiceTraffic (NodeContainer sourceNodes, Ipv4Address sinkAddress);
    ApplicationContainer GenerateConsumerDataTraffic (NodeContainer sourceNodes,
                                                      Ipv4Address sinkAddress,
                                                      bool isUplink);
    ApplicationContainer GeneratePortableDataTraffic (NodeContainer sourceNodes,
                                                      Ipv4Address sinkAddress,
                                                      bool isUplink);
    void AttachUserToBeam (uint16_t beamId,
                           Ptr<Node> userNode,
                           Ipv4Address destAddress,
                           TrafficModelType type,
                           bool isUplink);
    ApplicationContainer InstallSink (NodeContainer nodes);

    uint64_t GetNodeReceivedBytes (uint32_t nodeId, TrafficModelType type) const;

  private:
    struct VoipSessionState {
        bool legATalking {true};
        bool inMutualSilence {false};
        Time stateChangeTime {Time (0)};
        Ptr<UniformRandomVariable> talkSpurtRv;
        Ptr<UniformRandomVariable> turnGapRv;
    };

    uint32_t CalculateMaxPayloadBytesPerTti () const;
    uint32_t CalculateTcpWindowBytes () const;
    ApplicationContainer InstallHttpInternal (NodeContainer sourceNodes,
                                              Ipv4Address sinkAddress,
                                              bool isUplink,
                                              bool lightweightProfile);
    void ApplyTcpCongestionType (Ptr<Node> node) const;
    void OnQueueStateFeedback (double utilizationPercent);
    void AdjustRateForClosedLoop (double utilizationPercent);

    Ptr<Node> FindNodeByAddress (Ipv4Address address) const;
    Ipv4Address GetPrimaryIpv4Address (Ptr<Node> node) const;
    void RegisterNodeAddresses (NodeContainer nodes);
    void RegisterNodeAddresses (Ptr<Node> node);
    Ptr<ThreeGppHttpServer> EnsureHttpServer (Ptr<Node> node,
                                             Ipv4Address localAddress,
                                             bool lightweightProfile);
    void ConfigureHttpVariables (Ptr<ThreeGppHttpVariables> variables,
                                 bool lightweightProfile,
                                 bool uploadProfile = false) const;

    void RecordReceivedPacket (uint32_t nodeId, TrafficModelType type, Ptr<const Packet> packet);
    void NotifyPacketTx (std::string context, Ptr<const Packet> packet);
    void NotifyPacketRx (std::string context, Ptr<const Packet> packet, const Address& from);
    void NotifyPacketRxNoAddress (std::string context, Ptr<const Packet> packet);
    void MarkTrafficModelInstalled (TrafficModelType type);
    static const char* TrafficModelTypeToString (TrafficModelType type);

    TrafficModelType m_modelType {TRAFFIC_FULL_BUFFER};
    std::set<TrafficModelType> m_installedTypes;

    uint64_t m_targetRateBps {10'000'000};
    Time m_geoRtt {MilliSeconds (630)};
    uint32_t m_maxPrbPerTti {25};
    uint32_t m_rohcOverheadBytes {2};
    uint16_t m_mtu {1500};
    Time m_appStartTime {Seconds (1.0)};
    Time m_appStopTime {Seconds (100.0)};
    TypeId m_tcpCongestionTypeId;

    bool m_closedLoopControlEnabled {false};
    uint8_t m_lowWatermarkPercent {80};
    uint8_t m_highWatermarkPercent {95};
    double m_currentRateMultiplier {1.0};
    Callback<void, double> m_queueStateCallback;

    TrafficModelStats m_totalStats;
    std::map<TrafficModelType, TrafficModelStats> m_statsByType;
    Callback<void, Ptr<const Packet>, const Address&> m_txTraceCallback;

    struct FlowState {
        Ptr<Socket> socket;
        TrafficModelType type {TRAFFIC_FULL_BUFFER};
        uint32_t nodeId {0};
        uint32_t packetSize {0};
        Time baseInterval {Time (0)};
        bool active {true};
        bool voiceVadEnabled {false};
        bool voiceActive {true};
        bool isVoipLegA {false};
        uint32_t activePacketSize {0};
        uint32_t silencePacketSize {0};
        Time silenceInterval {Time (0)};
        Time stateChangeTime {Time (0)};
        Ptr<UniformRandomVariable> voiceTalkSpurtRv;
        Ptr<UniformRandomVariable> voiceSilenceRv;
        std::shared_ptr<VoipSessionState> voipSession;
        std::function<void (void)> driver;
        uint16_t rtpSeq {0};
        uint32_t rtpTimestamp {0};
        uint64_t bytesSent {0};
    };

    std::vector<std::shared_ptr<FlowState>> m_flows;
    uint64_t m_totalWindowBytes {0};
    Time m_totalWindowStart {Time (0)};
    std::map<uint32_t, uint64_t> m_totalWindowBytesByNode;
    std::map<uint32_t, Time> m_totalWindowStartByNode;
    std::map<uint32_t, double> m_totalFlowPeakBpsByNode;
    std::map<std::pair<TrafficModelType, uint32_t>, uint64_t> m_typeWindowBytesByFlow;
    std::map<std::pair<TrafficModelType, uint32_t>, Time> m_typeWindowStartByFlow;
    std::map<std::pair<TrafficModelType, uint32_t>, double> m_typeFlowPeakBpsByFlow;

    std::map<Ipv4Address, Ptr<Node>> m_nodesByAddress;
    std::map<uint32_t, Ptr<ThreeGppHttpServer>> m_httpServersByNodeId;
    std::map<TrafficModelType, std::map<uint32_t, uint64_t>> m_rxBytesByType;

    Time ScaledInterval (Time base) const;
    void AccountTx (FlowState& fs, uint32_t bytesSentOnWire, uint32_t usefulBytes);
    void AccountTx (TrafficModelType type, uint32_t nodeId, uint32_t bytesSentOnWire, uint32_t usefulBytes);
    void UpdateTxStats (TrafficModelStats& stats, uint32_t bytesSentOnWire, uint32_t usefulBytes, Time now);
    void UpdatePeakStats (TrafficModelStats& stats,
                          uint32_t nodeId,
                          uint64_t& windowBytes,
                          Time& windowStart,
                          double& flowPeakBytesPerSec,
                          uint32_t usefulBytes,
                          Time now);
    static constexpr uint16_t LEGACY_PORT = 9;
    static constexpr uint16_t FTP_PORT = 20;
    static constexpr uint16_t HTTP_PORT = 80;
    static constexpr uint16_t VOIP_PORT = 5000;
    static constexpr uint32_t TCP_HEADER_BYTES = 40;
    static constexpr uint32_t UDP_HEADER_BYTES = 28;
};

} // namespace ns3

#endif /* SAT_TRAFFIC_MODELS_H */
