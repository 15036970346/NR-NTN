/*
 * traffic-models.h
 *
 * Comprehensive traffic generation models based on 3GPP TR 37.901 V11.7.0 Clause 5.4.2
 * Implements: Full Buffer, FTP, HTTP, VoIP/RTP traffic models
 *
 * Key design goals:
 * - Saturation-oriented: designed to fill 25 PRB single-beam physical layer capacity
 * - BDP-aware TCP: TCP window sized for GEO RTT (600ms+ RTT)
 * - Closed-loop rate control: MAC queue state monitoring for adaptive transmission
 */

#ifndef SAT_TRAFFIC_MODELS_H
#define SAT_TRAFFIC_MODELS_H

#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/object.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"
#include <map>
#include <memory>

namespace ns3 {

/**
 * \enum TrafficModelType
 * \brief Traffic model types per 3GPP TR 37.901 Clause 5.4.2
 */
enum TrafficModelType {
    TRAFFIC_FULL_BUFFER,     //!< Full Buffer - peak throughput test (saturation testing)
    TRAFFIC_FTP,            //!< FTP - TCP-based file transfer
    TRAFFIC_HTTP,           //!< HTTP - TCP-based web browsing (bursty)
    TRAFFIC_VOIP_RTP,       //!< VoIP - UDP/RTP real-time voice
    TRAFFIC_LEGACY_DATA,    //!< Legacy consumer data (backward compatible)
    TRAFFIC_LEGACY_VOICE,   //!< Legacy voice (backward compatible)
};

/**
 * \struct TrafficModelStats
 * \brief Statistics for traffic model performance monitoring
 */
struct TrafficModelStats {
    uint64_t totalBytesGenerated {0};    //!< Total bytes generated at app layer (L5+ payload)
    uint64_t totalPacketsGenerated {0};  //!< Total packets generated
    uint64_t totalBytesSent {0};         //!< Total bytes actually sent (after socket)
    uint64_t usefulBytesDelivered {0};   //!< Useful user bytes (excluding L2-L4 headers) per TR 37.901 Clause 5.1.2
    uint64_t bufferUnderrunCount {0};    //!< Buffer underrun events (source couldn't fill fast enough)
    Time lastGenerateTime {Time (0)};    //!< Last packet generation timestamp
    double avgGenerateIntervalUs {0};     //!< Average inter-packet interval (microseconds)
    double peakThroughputBps {0};        //!< Peak 1-second throughput (bits/s) per user
    double beamPeakThroughputBps {0};    //!< Peak beam-aggregate 1-second throughput (bits/s)
    uint32_t ipv4Packets {0};            //!< Number of IPv4 packets
    uint32_t ipv6Packets {0};            //!< Number of IPv6 packets (future)
    std::map<uint32_t, uint64_t> perUserUsefulBytes; //!< node id -> useful bytes
    std::map<uint32_t, double>   perUserPeakBps;     //!< node id -> peak Bps
};

/**
 * \class SatTrafficGenerator
 * \brief Advanced traffic generator with 4 traffic models per 3GPP TR 37.901
 *
 * This class implements:
 * 1. Full Buffer: Always keeps buffer full, for peak throughput testing
 * 2. FTP: TCP large file transfer, BDP-aligned window
 * 3. HTTP: TCP bursty traffic, statistical distribution modeling
 * 4. VoIP/RTP: UDP real-time voice, RTP encapsulation
 *
 * Performance optimization:
 * - Closed-loop rate control: Monitor MAC queue state, auto-adjust发包频率
 * - Packet size mapping: 25 PRB capacity calculation, maximize single packet
 * - ROHC-aware: Consider compressed header overhead in payload calculation
 */
class SatTrafficGenerator : public Object
{
public:
    static TypeId GetTypeId (void);

    SatTrafficGenerator ();
    virtual ~SatTrafficGenerator ();

    // ========== Configuration ==========

    /**
     * \brief Set traffic model type
     * \param type TRAFFIC_FULL_BUFFER | TRAFFIC_FTP | TRAFFIC_HTTP | TRAFFIC_VOIP_RTP
     */
    void SetTrafficModel (TrafficModelType type);

    /**
     * \brief Get current traffic model type
     */
    TrafficModelType GetTrafficModel () const { return m_modelType; }

    /**
     * \brief Set target data rate (for Full Buffer mode)
     * \param rateBitsPerSec Target rate (bits/s)
     */
    void SetTargetRate (uint64_t rateBitsPerSec);

    /**
     * \brief Set GEO satellite RTT (for TCP window calculation)
     * \param rtt RTT delay
     */
    void SetGeoRtt (Time rtt);

    /**
     * \brief Set max PRBs per TTI (for packet size calculation)
     * \param numPrb PRB count
     */
    void SetMaxPrbPerTti (uint32_t numPrb);

    /**
     * \brief Set ROHC compressed header size (for net payload calculation)
     * \param bytes Compressed header bytes
     */
    void SetRohcOverheadBytes (uint32_t bytes);

    /**
     * \brief Set application start/stop time used by subsequently installed flows.
     */
    void SetApplicationWindow (Time start, Time stop);

    /**
     * \brief Enable/disable closed-loop rate control (MAC queue monitoring)
     * \param enable true to enable
     * \param lowWatermarkPercent Low watermark threshold (default 80%)
     */
    void EnableClosedLoopControl (bool enable, uint8_t lowWatermarkPercent = 80);

    // ========== Application Installation ==========

    /**
     * \brief Install Full Buffer traffic (peak throughput test)
     *
     * Full Buffer characteristics:
     * - Buffer always in "full" state
     * - Once RLC requests data, immediately fill
     * - Target: fill 25 PRB single-beam downlink capacity
     * - Target rate: 10Mbps+ (single user)
     */
    ApplicationContainer InstallFullBuffer (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                           bool isUplink = false);

    /**
     * \brief Install FTP traffic (TCP-based large file transfer)
     *
     * FTP characteristics (3GPP TR 37.901 Clause 5.4.2.2):
     * - TCP window > Bandwidth-Delay Product (BDP)
     * - File size: 2GB for static 60s sim, 5GB for fading 164s sim
     * - MTU: 1280-1500 bytes
     * - Window Scaling enabled
     */
    ApplicationContainer InstallFtp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                     bool isUplink = false);

    /**
     * \brief Install HTTP traffic (TCP-based bursty web browsing)
     *
     * HTTP characteristics (3GPP TR 37.901 Clause 5.4.2.3):
     * - Simulates page parsing delay, main object/embedded object sizes
     * - Uses statistical distributions: Pareto/Exponential
     * - Exhibits bursty traffic pattern
     */
    ApplicationContainer InstallHttp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                      bool isUplink = false);

    /**
     * \brief Install VoIP/RTP traffic (UDP real-time voice)
     *
     * VoIP characteristics (3GPP TR 37.901 Clause 5.4.2.4):
     * - Maintains original 2.4Kbps logic
     * - RTP protocol encapsulation (UDP transport)
     * - Small packets: 40-60 bytes (voice + RTP header)
     * - GEO long delay environment adaptation
     */
    ApplicationContainer InstallVoipRtp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                          bool isUplink = false);

    /**
     * \brief Install Legacy consumer data traffic (backward compatible)
     */
    ApplicationContainer InstallLegacyConsumerData (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                                    bool isUplink = false);

    // ========== Traffic Statistics ==========

    /**
     * \brief Get traffic statistics
     */
    TrafficModelStats GetStats () const { return m_stats; }

    /**
     * \brief Reset statistics
     */
    void ResetStats ();

    /**
     * \brief Export statistics to file
     */
    void DumpStats (std::ostream& os) const;

    // ========== Callbacks ==========

    /**
     * \brief Set MAC queue state callback (for closed-loop control)
     * Callback signature: void (queue_utilization_percent)
     */
    void SetQueueStateCallback (Callback<void, double> cb);

    /**
     * \brief Register Application Tx callback (for statistics)
     */
    void SetTxTraceCallback (Callback<void, Ptr<const Packet>, const Address&> cb);

    // ========== Compatibility Interfaces ==========

    ApplicationContainer GenerateVoiceTraffic (NodeContainer sourceNodes, Ipv4Address sinkAddress);
    ApplicationContainer GenerateConsumerDataTraffic (NodeContainer sourceNodes, Ipv4Address sinkAddress, bool isUplink);
    ApplicationContainer GeneratePortableDataTraffic (NodeContainer sourceNodes, Ipv4Address sinkAddress, bool isUplink);
    void AttachUserToBeam (uint16_t beamId, Ptr<Node> userNode, Ipv4Address destAddress, TrafficModelType type, bool isUplink);
    /**
     * \brief Install all required sinks for the supported TR 37.901 traffic models.
     *
     * Callers should install sinks on the actual receiving nodes before creating
     * flows. Traffic model installers only create source-side applications.
     */
    ApplicationContainer InstallSink (NodeContainer nodes);

private:
    // ========== Internal Methods ==========

    /**
     * \brief Calculate max payload bytes per TTI for Full Buffer
     *
     * Based on:
     * - 25 PRB x 12 subcarriers x 14 symbols x 6 bits (64-QAM) / TTI
     * - Minus ROHC compressed header overhead
     * - Minus TCP/IP headers (40 bytes)
     */
    uint32_t CalculateMaxPayloadBytesPerTti () const;

    /**
     * \brief Calculate TCP BDP window size
     *
     * BDP = bandwidth x RTT
     * GEO: 单波束 25 PRB, RTT=630ms -> BDP ~ 15.9Mbits = 1.98MB
     * TCP window should be 2-3x BDP to fully utilize link
     */
    uint32_t CalculateTcpWindowBytes () const;

    /**
     * \brief Handle MAC queue state feedback (closed-loop control)
     */
    void OnQueueStateFeedback (double utilizationPercent);

    /**
     * \brief Adjust transmission rate based on queue utilization
     */
    void AdjustRateForClosedLoop (double utilizationPercent);

    // ========== Member Variables ==========

    TrafficModelType m_modelType {TRAFFIC_FULL_BUFFER};

    // Performance parameters
    uint64_t m_targetRateBps {10'000'000};    //!< Target rate 10Mbps
    Time m_geoRtt {MilliSeconds (630)};        //!< GEO RTT (600ms + 30ms processing)
    uint32_t m_maxPrbPerTti {25};             //!< Max PRBs per TTI (single beam)
    uint32_t m_rohcOverheadBytes {2};         //!< ROHC compressed header (typical 2 bytes)
    uint16_t m_mtu {1500};                     //!< MTU
    Time m_appStartTime {Seconds (1.0)};       //!< Absolute application start time
    Time m_appStopTime {Seconds (100.0)};      //!< Absolute application stop time

    // Closed-loop rate control
    bool m_closedLoopControlEnabled {false};
    uint8_t m_lowWatermarkPercent {80};
    uint8_t m_highWatermarkPercent {95};
    double m_currentRateMultiplier {1.0};      //!< Current rate multiplier (1.0=normal, 2.0=accelerated)
    Callback<void, double> m_queueStateCallback;
    EventId m_closedLoopAdjustmentEvent;

    // Statistics
    TrafficModelStats m_stats;
    Callback<void, Ptr<const Packet>, const Address&> m_txTraceCallback;
    uint16_t m_port {9};  //!< Legacy port for backward compatibility

    // Internal state
    std::map<Ptr<Socket>, uint64_t> m_socketTxBytes;
    std::map<Ptr<Socket>, EventId> m_pendingTxEvents;
    std::map<Ptr<Socket>, bool> m_socketConnected;

    /**
     * \brief Per-flow runtime state used by recursive transmission loops.
     *
     * Held via shared_ptr and captured by value in std::function closures so
     * the state outlives the Install* method that creates it (fixing the
     * previous reference-capture dangling-pointer bug).
     */
    struct FlowState {
        Ptr<Socket> socket;
        uint32_t nodeId {0};
        uint32_t packetSize {0};          //!< Application-layer payload size
        uint32_t headerOverheadBytes {0}; //!< TCP/UDP/IP/RTP header bytes per packet (not sent, used to adjust useful bytes)
        uint64_t bytesSent {0};
        uint64_t fileSizeBytes {0};       //!< FTP target; 0 = unbounded
        Time     baseInterval {Time (0)}; //!< Base inter-packet interval
        uint16_t rtpSeq {0};
        uint32_t rtpTimestamp {0};
        uint32_t pageCount {0};
        bool     active {true};
        // Windowed peak tracking (rolling 1-second window)
        Time     windowStart {Time (0)};
        uint64_t windowBytes {0};
        double   peakBytesPerSec {0};
        std::function<void(void)> driver; //!< Recursive tx driver (captures shared_ptr to self)
    };

    std::vector<std::shared_ptr<FlowState>> m_flows; //!< Keeps flow state alive for simulation lifetime
    uint64_t m_totalWindowBytes {0};                  //!< Aggregate across flows for beam-peak tracking
    Time     m_totalWindowStart {Time (0)};

    /** Apply closed-loop multiplier to a base interval. */
    Time ScaledInterval (Time base) const;

    /** Update stats for a successful Tx, including per-user + peak tracking. */
    void AccountTx (FlowState& fs, uint32_t bytesSentOnWire, uint32_t usefulBytes);

    // Constants
    static constexpr uint32_t TCP_HEADER_BYTES = 40;        //!< TCP+IP header
    static constexpr uint32_t UDP_HEADER_BYTES = 28;       //!< UDP+IP header
    static constexpr uint32_t RTP_HEADER_BYTES = 12;        //!< RTP header
    static constexpr uint32_t IPv4_HEADER_BYTES = 20;       //!< IPv4 header
    static constexpr uint32_t MAX_RTP_PAYLOAD_VOIP = 244;   //!< RFC3550 max RTP payload for VoIP
};

} // namespace ns3

#endif /* SAT_TRAFFIC_MODELS_H */
