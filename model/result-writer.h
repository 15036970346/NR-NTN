/*
 * 文件路径：contrib/geo-sat/model/result-writer.h
 * 功能：NR NTN仿真结果输出管理器
 * 将仿真数据输出到指定目录的txt文件
 */
#ifndef RESULT_WRITER_H
#define RESULT_WRITER_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <string>
#include <map>
#include <fstream>

namespace ns3 {

class ResultWriter : public Object
{
public:
    static TypeId GetTypeId (void);
    ResultWriter ();
    virtual ~ResultWriter ();

    // 设置输出目录
    void SetOutputDirectory (std::string dirPath);
    std::string GetOutputDirectory () const;

    // 创建输出文件
    void OpenFile (std::string filename);
    void CloseFile (std::string filename);

    // 写入接口
    void WriteAccessStats (uint32_t totalAttempts, uint32_t successCount, 
                          uint32_t collisionCount, uint32_t timeoutCount,
                          double successRate);
    
    void WriteHarqStats (uint32_t totalTx, uint32_t retransmissions,
                         uint32_t ackCount, uint32_t nackCount,
                         double retransmissionRate);
    
    void WriteUserRateStats (uint16_t rnti, double peakRate, double avgRate,
                            uint64_t totalTxBytes, uint64_t totalRxBytes);
    
    void WriteBeamStats (uint32_t beamId, uint32_t activeUes,
                        uint64_t throughputBits, double avgCqi,
                        double spectrumEfficiency);

    // 功率域 NOMA 复用记账: 每波束累计共享(复用)RB、累计已用RB、复用增益比例。
    // reuseGain = sharedRbs / usedRbs, 表示功率域复用等效额外提供的 RB 占主用户已用RB比例。
    void WriteNomaReuseStats (const std::string& filename, uint32_t beamId,
                             uint64_t sharedRbs, uint64_t usedRbs, double reuseGain);
    
    void WriteSystemStats (uint64_t totalCapacity, double peakRate,
                          double avgRate, double spectrumEfficiency);
    
    void WriteFrequencyReuseConfig (uint8_t reuseFactor, uint32_t totalRbs,
                                   uint32_t rbsPerBeam, double reuseGain);
    
    void WriteSimulationConfig (double bandwidthHz, uint32_t ulRbs, uint32_t dlRbs,
                               uint8_t scs, uint32_t numBeams, uint32_t numUes,
                               Time simDuration);
    
    // 通用写入
    void WriteString (std::string filename, std::string content);
    void AppendString (std::string filename, std::string content);

private:
    std::string GetFullPath (std::string filename) const;
    void EnsureDirectoryExists ();
    std::string GetTimestamp () const;

    std::string m_outputDirectory;
    std::map<std::string, std::ofstream*> m_openFiles;
};

} // namespace ns3

#endif /* RESULT_WRITER_H */
