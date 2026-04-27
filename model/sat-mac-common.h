/*
 * 文件路径：contrib/geo-sat/model/sat-mac-common.h
 * 功能：GEO 卫星 MAC 层公共数据结构定义 (供全组模块调用)
 */
#ifndef SAT_MAC_COMMON_H
#define SAT_MAC_COMMON_H

#include "ns3/buffer.h"
#include "ns3/header.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <stdint.h>

namespace ns3 {

// 1. 测量配置结构体 (用于RRC层)
struct MeasConfig {
    double rsrpThreshold;    // RSRP门限 (dBm)
    double hysteresis;       // 迟滞 (dB)
    Time timeToTrigger;      // Time-to-Trigger定时器
    double filterCoeff;      // L3滤波系数
};

// 2. 测量报告结构体 (用于崔博开的测量模块与秦宇航的调度器交互)
struct MeasReport {
    uint16_t ueId;           // 用户标识
    uint32_t bestBeamId;     // 测量到的最强波束 ID
    double rsrp;             // 接收信号参考功率
    Vector position;         // UE位置 (星地切换用)
};

// 2. 下行控制信息结构体 (用于秦宇航的调度器下发给你谢昀松的终端 MAC 层)
struct DciInfo {
    bool isUplinkGrant;      // 是否为上行授权
    uint32_t rbAllocation;   // 分配的 RB 数量
    uint8_t mcs;             // 调制编码策略
    double txPowerDbm;       // 上行授权对应的目标发射功率 (DL调度时置0)
    Time delayToStart;       // 调度延迟 (K2)
    Time duration;           // 授权持续时间
};

// 3. PUCCH信息结构体 (上行控制信息 - UCI)
enum class PucchFormatType {
    FORMAT_0,  // SR (Scheduling Request) - 1bit
    FORMAT_1,  // CSI/CQI - 宽带或波束指示
    FORMAT_2,  // HARQ ACK/NACK - 1-2bit
    FORMAT_3,  // HARQ ACK + CSI
    FORMAT_4   // 多用户PUSCH
};

struct PucchInfo {
    PucchFormatType format;   // PUCCH格式
    uint16_t rnti;           // 用户标识
    bool srPending;          // SR挂起标志
    uint8_t cqi;             // CQI值 (1-15)
    bool harqAck;            // HARQ ACK/NACK (true=ACK, false=NACK)
    uint8_t harqBitMap;      // 多bit HARQ (如多TB)
    Time transmissionTime;   // 传输时间
};

// 4. PUSCH信息结构体 (上行共享信道 - 含BSR)
struct PuschInfo {
    uint16_t rnti;           // 用户标识
    uint32_t rbAllocation;   // 分配的RB数量
    uint8_t mcs;             // 调制编码策略
    uint32_t bufferStatus;   // BSR缓冲区状态 (bytes)
    bool isNewTransmission;  // 新传还是重传
    Time delayToStart;       // 传输延迟
    Time duration;           // 持续时间
};

// 5. BSR MAC CE结构体 (缓冲区状态报告)
struct BsR_MAC_CE {
    uint16_t rnti;           // 用户标识
    uint8_t lcgId;           // 逻辑信道组ID
    uint32_t bufferSize;     // 缓冲区大小 (bytes)
};

// 6. PRACH前导码结构体 (随机接入)
struct PrachPreamble {
    uint16_t rnti;           // 用户标识 (如果已分配)
    uint32_t preambleId;      // 前导码ID (0-63)
    uint8_t format;          // PRACH格式 (0-4)
    uint8_t utType;          // 终端类型 (0=portable, 1=consumer)
    Time transmissionTime;    // 传输时间
    bool isRetransmission;   // 是否为重传
};

// 7. SSB同步信号块结构体
struct SsbBlock {
    uint32_t beamId;         // 波束ID
    uint32_t pci;            // 物理小区ID
    uint8_t ssbIndex;        // SSB索引 (0-63)
    double absoluteFrequencySsb; // SSB绝对频点
    Time transmissionTime;   // 传输时间
    double rsrp;             // RSRP测量值
};

// ==================== 4 步随机接入 (Msg1~Msg4) ====================
// Msg1 即 PrachPreamble (见上方结构体 6)。
// Msg2 (RAR)：基站对每个收到的 preambleId 生成一次 RAR；若多个 UE 选用同一个
// preambleId，它们都能"听到"同一个 RAR 并解析同一份 TC-RNTI，这样才能进入
// Msg3 阶段并在 Msg3 上发生真正的竞争（竞争解决）。
struct RarMessage {
    uint32_t raRnti;              // 由 PRACH 时频资源派生
    uint32_t preambleId;          // 对应的前导码 ID (UE 用于匹配)
    uint16_t tcRnti;              // 分配的临时 C-RNTI
    Time     timingAdvance;       // 定时提前量 (GEO ~300 ms)
    uint32_t ulGrantRbs;          // Msg3 的 UL Grant (PRB 数)
    uint8_t  ulGrantMcs;          // Msg3 的 MCS
    double   ulGrantTxPowerDbm;   // Msg3 的目标发射功率
    Time     msg3DelayToStart;    // Msg3 相对 RAR 的触发延迟 (K2)
    Time     transmissionTime;    // 发送时间戳
};

// Msg3: RRCSetupRequest (UE → gNB, 承载在 PUSCH 上)
// 终端在 DciInfo 对应的资源上发送，基站通过 tcRnti 寻址；ueIdentity 用于后续
// Msg4 的竞争解决 (contention resolution identity echo)
struct RrcSetupRequest {
    uint16_t tcRnti;              // RAR 中收到的临时 C-RNTI
    uint64_t ueIdentity;          // UE 随机身份 (40-bit S-TMSI 或 random value)
    uint8_t  establishmentCause;  // 0=mt-Access 1=mo-Signalling 2=mo-Data ...
    uint32_t preambleIdUsed;      // 发起本次接入时使用的 PreambleId (调试/追踪)
    Time     transmissionTime;    // 发送时间戳
};

class GenericUlMacHeader : public Header
{
public:
    static TypeId GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::GenericUlMacHeader")
            .SetParent<Header> ()
            .SetGroupName ("SatGeo")
            .AddConstructor<GenericUlMacHeader> ();
        return tid;
    }

    TypeId GetInstanceTypeId () const override
    {
        return GetTypeId ();
    }

    uint32_t GetSerializedSize () const override
    {
        return 14;
    }

    void Serialize (Buffer::Iterator start) const override
    {
        start.WriteU16 (m_rnti);
        start.WriteU32 (m_payloadBytes);
        start.WriteU64 (m_transmissionTimeNs);
    }

    uint32_t Deserialize (Buffer::Iterator start) override
    {
        m_rnti = start.ReadU16 ();
        m_payloadBytes = start.ReadU32 ();
        m_transmissionTimeNs = start.ReadU64 ();
        return GetSerializedSize ();
    }

    void Print (std::ostream& os) const override
    {
        os << "rnti=" << m_rnti
           << " payloadBytes=" << m_payloadBytes
           << " txNs=" << m_transmissionTimeNs;
    }

    void SetRnti (uint16_t rnti)
    {
        m_rnti = rnti;
    }

    uint16_t GetRnti () const
    {
        return m_rnti;
    }

    void SetPayloadBytes (uint32_t payloadBytes)
    {
        m_payloadBytes = payloadBytes;
    }

    uint32_t GetPayloadBytes () const
    {
        return m_payloadBytes;
    }

    void SetTransmissionTime (Time txTime)
    {
        m_transmissionTimeNs = static_cast<uint64_t> (txTime.GetNanoSeconds ());
    }

    Time GetTransmissionTime () const
    {
        return NanoSeconds (m_transmissionTimeNs);
    }

private:
    uint16_t m_rnti {0};
    uint32_t m_payloadBytes {0};
    uint64_t m_transmissionTimeNs {0};
};

class Msg3MacHeader : public Header
{
public:
    static TypeId GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::Msg3MacHeader")
            .SetParent<Header> ()
            .SetGroupName ("SatGeo")
            .AddConstructor<Msg3MacHeader> ();
        return tid;
    }

    TypeId GetInstanceTypeId () const override
    {
        return GetTypeId ();
    }

    uint32_t GetSerializedSize () const override
    {
        return 23;
    }

    void Serialize (Buffer::Iterator start) const override
    {
        start.WriteU16 (m_tcRnti);
        start.WriteU64 (m_ueIdentity);
        start.WriteU8 (m_establishmentCause);
        start.WriteU32 (m_preambleIdUsed);
        start.WriteU64 (m_transmissionTimeNs);
    }

    uint32_t Deserialize (Buffer::Iterator start) override
    {
        m_tcRnti = start.ReadU16 ();
        m_ueIdentity = start.ReadU64 ();
        m_establishmentCause = start.ReadU8 ();
        m_preambleIdUsed = start.ReadU32 ();
        m_transmissionTimeNs = start.ReadU64 ();
        return GetSerializedSize ();
    }

    void Print (std::ostream& os) const override
    {
        os << "tcRnti=" << m_tcRnti
           << " ueIdentity=" << m_ueIdentity
           << " cause=" << static_cast<uint32_t> (m_establishmentCause)
           << " preambleId=" << m_preambleIdUsed
           << " txNs=" << m_transmissionTimeNs;
    }

    void SetRequest (const RrcSetupRequest& req)
    {
        m_tcRnti = req.tcRnti;
        m_ueIdentity = req.ueIdentity;
        m_establishmentCause = req.establishmentCause;
        m_preambleIdUsed = req.preambleIdUsed;
        m_transmissionTimeNs = static_cast<uint64_t> (req.transmissionTime.GetNanoSeconds ());
    }

    RrcSetupRequest ToRequest () const
    {
        RrcSetupRequest req;
        req.tcRnti = m_tcRnti;
        req.ueIdentity = m_ueIdentity;
        req.establishmentCause = m_establishmentCause;
        req.preambleIdUsed = m_preambleIdUsed;
        req.transmissionTime = NanoSeconds (m_transmissionTimeNs);
        return req;
    }

private:
    uint16_t m_tcRnti {0};
    uint64_t m_ueIdentity {0};
    uint8_t m_establishmentCause {0};
    uint32_t m_preambleIdUsed {0};
    uint64_t m_transmissionTimeNs {0};
};

// Msg4: RRCSetup (gNB → UE) —— 完成竞争解决
// 基站在 Msg4 中回显 ueIdentity；UE 将其与本地身份比对，匹配即抢占成功，
// 并将 TC-RNTI 晋升为正式 C-RNTI；不匹配视作竞争失败。
struct RrcSetupMessage {
    uint16_t tcRnti;              // 对应 Msg2/Msg3 的临时 C-RNTI
    uint64_t echoedUeIdentity;    // 回显 Msg3 中的 ueIdentity (竞争解决 ID)
    uint16_t cRnti;               // 晋升后的正式 C-RNTI
    Time     transmissionTime;    // 发送时间戳
};

} // namespace ns3

#endif /* SAT_MAC_COMMON_H */
