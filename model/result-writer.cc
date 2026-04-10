/*
 * 文件路径：contrib/geo-sat/model/result-writer.cc
 * 功能：NR NTN仿真结果输出管理器实现
 */
#include "result-writer.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ResultWriter");
NS_OBJECT_ENSURE_REGISTERED (ResultWriter);

TypeId ResultWriter::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ResultWriter")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<ResultWriter> ();
  return tid;
}

ResultWriter::ResultWriter ()
{
  NS_LOG_FUNCTION (this);
  m_outputDirectory = "./simulation_results";
}

ResultWriter::~ResultWriter ()
{
  NS_LOG_FUNCTION (this);
  
  for (auto& pair : m_openFiles) {
      if (pair.second && pair.second->is_open ()) {
          pair.second->close ();
          delete pair.second;
      }
  }
  m_openFiles.clear ();
}

void ResultWriter::SetOutputDirectory (std::string dirPath)
{
  m_outputDirectory = dirPath;
  EnsureDirectoryExists ();
  NS_LOG_INFO ("ResultWriter: Output directory set to " << m_outputDirectory);
}

std::string ResultWriter::GetOutputDirectory () const
{
  return m_outputDirectory;
}

void ResultWriter::EnsureDirectoryExists ()
{
  mkdir (m_outputDirectory.c_str (), 0755);
}

std::string ResultWriter::GetFullPath (std::string filename) const
{
  return m_outputDirectory + "/" + filename;
}

std::string ResultWriter::GetTimestamp () const
{
  std::stringstream ss;
  Time now = Simulator::Now ();
  ss << "t" << std::fixed << std::setprecision (3) << now.GetSeconds () << "s";
  return ss.str ();
}

void ResultWriter::OpenFile (std::string filename)
{
  std::string fullPath = GetFullPath (filename);
  
  if (m_openFiles.find (filename) != m_openFiles.end ()) {
      if (m_openFiles[filename]->is_open ()) {
          return;
      }
  }
  
  std::ofstream* ofs = new std::ofstream (fullPath, std::ios::out);
  if (ofs->is_open ()) {
      m_openFiles[filename] = ofs;
      NS_LOG_INFO ("ResultWriter: Opened file " << fullPath);
  } else {
      NS_LOG_ERROR ("ResultWriter: Failed to open file " << fullPath);
      delete ofs;
  }
}

void ResultWriter::CloseFile (std::string filename)
{
  auto it = m_openFiles.find (filename);
  if (it != m_openFiles.end () && it->second->is_open ()) {
      it->second->close ();
      delete it->second;
      m_openFiles.erase (it);
      NS_LOG_INFO ("ResultWriter: Closed file " << filename);
  }
}

void ResultWriter::WriteAccessStats (uint32_t totalAttempts, uint32_t successCount,
                                   uint32_t collisionCount, uint32_t timeoutCount,
                                   double successRate)
{
  std::string filename = "access_statistics.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << "# NR NTN Access Statistics Report\n";
  ofs << "# Generated at simulation time: " << GetTimestamp () << "\n";
  ofs << "# =========================================\n\n";
  ofs << "Total_Access_Attempts: " << totalAttempts << "\n";
  ofs << "Successful_Access: " << successCount << "\n";
  ofs << "Collision_Failures: " << collisionCount << "\n";
  ofs << "Timeout_Failures: " << timeoutCount << "\n";
  ofs << "Access_Success_Rate: " << std::fixed << std::setprecision (4) 
      << (successRate * 100) << "%\n";
  ofs << "\n# =========================================\n";
  ofs << "# End of Report\n";
  
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: Access statistics written to " << fullPath);
}

void ResultWriter::WriteHarqStats (uint32_t totalTx, uint32_t retransmissions,
                                  uint32_t ackCount, uint32_t nackCount,
                                  double retransmissionRate)
{
  std::string filename = "harq_statistics.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << "# NR NTN HARQ Statistics Report\n";
  ofs << "# Generated at simulation time: " << GetTimestamp () << "\n";
  ofs << "# =========================================\n\n";
  ofs << "Total_Transmissions: " << totalTx << "\n";
  ofs << "First_Transmissions: " << (totalTx - retransmissions) << "\n";
  ofs << "Retransmissions: " << retransmissions << "\n";
  ofs << "ACK_Count: " << ackCount << "\n";
  ofs << "NACK_Count: " << nackCount << "\n";
  ofs << "Retransmission_Rate: " << std::fixed << std::setprecision (4) 
      << (retransmissionRate * 100) << "%\n";
  ofs << "HARQ_Success_Rate: " << std::fixed << std::setprecision (4) 
      << ((double)ackCount / totalTx * 100) << "%\n";
  ofs << "\n# =========================================\n";
  ofs << "# End of Report\n";
  
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: HARQ statistics written to " << fullPath);
}

void ResultWriter::WriteUserRateStats (uint16_t rnti, double peakRate, double avgRate,
                                      uint64_t totalTxBytes, uint64_t totalRxBytes)
{
  std::string filename = "user_rate_statistics.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::app);
  if (!ofs.is_open ()) {
      ofs.open (fullPath, std::ios::out);
      if (!ofs.is_open ()) {
          NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
          return;
      }
      ofs << "# NR NTN User Rate Statistics Report\n";
      ofs << "# =========================================\n";
      ofs << "# RNTI | Peak_Rate_Mbps | Avg_Rate_Mbps | Total_Tx_Bytes | Total_Rx_Bytes\n";
  }
  
  ofs << rnti << " | " 
      << std::fixed << std::setprecision (4) << (peakRate / 1e6) << " | "
      << std::fixed << std::setprecision (4) << (avgRate / 1e6) << " | "
      << totalTxBytes << " | "
      << totalRxBytes << "\n";
  
  ofs.close ();
  
  NS_LOG_DEBUG ("ResultWriter: User rate stat written for UE" << rnti);
}

void ResultWriter::WriteBeamStats (uint32_t beamId, uint32_t activeUes,
                                 uint64_t throughputBits, double avgCqi,
                                 double spectrumEfficiency)
{
  std::string filename = "beam_statistics.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::app);
  if (!ofs.is_open ()) {
      ofs.open (fullPath, std::ios::out);
      if (!ofs.is_open ()) {
          NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
          return;
      }
      ofs << "# NR NTN Beam Statistics Report\n";
      ofs << "# =========================================\n";
      ofs << "# Beam_ID | Active_UEs | Throughput_Mbps | Avg_CQI | Spectrum_Efficiency_bps_Hz\n";
  }
  
  ofs << beamId << " | "
      << activeUes << " | "
      << std::fixed << std::setprecision (4) << (throughputBits / 1e6) << " | "
      << std::fixed << std::setprecision (2) << avgCqi << " | "
      << std::fixed << std::setprecision (6) << spectrumEfficiency << "\n";
  
  ofs.close ();
}

void ResultWriter::WriteSystemStats (uint64_t totalCapacity, double peakRate,
                                    double avgRate, double spectrumEfficiency)
{
  std::string filename = "system_statistics.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << "# NR NTN System Statistics Report\n";
  ofs << "# Generated at simulation time: " << GetTimestamp () << "\n";
  ofs << "# =========================================\n\n";
  ofs << "Total_System_Capacity: " << (totalCapacity / 1e6) << " Mbps\n";
  ofs << "System_Peak_Rate: " << std::fixed << std::setprecision (4) 
      << (peakRate / 1e6) << " Mbps\n";
  ofs << "System_Average_Rate: " << std::fixed << std::setprecision (4) 
      << (avgRate / 1e6) << " Mbps\n";
  ofs << "Spectrum_Efficiency: " << std::fixed << std::setprecision (6) 
      << spectrumEfficiency << " bps/Hz\n";
  ofs << "\n# =========================================\n";
  ofs << "# End of Report\n";
  
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: System statistics written to " << fullPath);
}

void ResultWriter::WriteFrequencyReuseConfig (uint8_t reuseFactor, uint32_t totalRbs,
                                            uint32_t rbsPerBeam, double reuseGain)
{
  std::string filename = "frequency_reuse_config.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << "# NR NTN Frequency Reuse Configuration\n";
  ofs << "# Generated at simulation time: " << GetTimestamp () << "\n";
  ofs << "# =========================================\n\n";
  ofs << "Reuse_Factor: 1/" << (uint32_t)reuseFactor << " (7-color)\n";
  ofs << "Total_RBs: " << totalRbs << "\n";
  ofs << "RBs_Per_Beam: " << rbsPerBeam << "\n";
  ofs << "Reuse_Gain_dB: " << std::fixed << std::setprecision (2) << reuseGain << " dB\n";
  ofs << "\n# Color Mapping:\n";
  ofs << "# Color 0 (Red):     Beam 0, 7, 14...\n";
  ofs << "# Color 1 (Green):   Beam 1, 8, 15...\n";
  ofs << "# Color 2 (Blue):    Beam 2, 9, 16...\n";
  ofs << "# Color 3 (Yellow):  Beam 3, 10, 17...\n";
  ofs << "# Color 4 (Purple):  Beam 4, 11, 18...\n";
  ofs << "# Color 5 (Cyan):    Beam 5, 12, 19...\n";
  ofs << "# Color 6 (Orange):  Beam 6, 13, 20...\n";
  ofs << "\n# =========================================\n";
  ofs << "# End of Report\n";
  
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: Frequency reuse config written to " << fullPath);
}

void ResultWriter::WriteSimulationConfig (double bandwidthHz, uint32_t ulRbs, uint32_t dlRbs,
                                        uint8_t scs, uint32_t numBeams, uint32_t numUes,
                                        Time simDuration)
{
  std::string filename = "simulation_config.txt";
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << "# NR NTN Simulation Configuration\n";
  ofs << "# Generated at simulation time: " << GetTimestamp () << "\n";
  ofs << "# =========================================\n\n";
  ofs << "Bandwidth_Hz: " << bandwidthHz << "\n";
  ofs << "Bandwidth_MHz: " << std::fixed << std::setprecision (2) << (bandwidthHz / 1e6) << "\n";
  ofs << "Subcarrier_Spacing_kHz: " << (uint32_t)scs << "\n";
  ofs << "UL_RBs_Limited: " << ulRbs << " (终端发射功率受限)\n";
  ofs << "DL_RBs_Full: " << dlRbs << "\n";
  ofs << "Number_of_Beams: " << numBeams << "\n";
  ofs << "Number_of_UEs: " << numUes << "\n";
  ofs << "Simulation_Duration_s: " << std::fixed << std::setprecision (3) 
      << simDuration.GetSeconds () << "\n";
  ofs << "\n# System Parameters:\n";
  ofs << "# Frequency_Band: S-band (2 GHz)\n";
  ofs << "# Satellite_Type: GEO (35786 km altitude)\n";
  ofs << "# RTT_Delay: ~600 ms\n";
  ofs << "# HARQ_Processes: 8 per UE\n";
  ofs << "# Max_Retransmissions: 4\n";
  ofs << "\n# =========================================\n";
  ofs << "# End of Report\n";
  
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: Simulation config written to " << fullPath);
}

void ResultWriter::WriteString (std::string filename, std::string content)
{
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::out);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << content;
  ofs.close ();
  
  NS_LOG_INFO ("ResultWriter: String written to " << fullPath);
}

void ResultWriter::AppendString (std::string filename, std::string content)
{
  std::string fullPath = GetFullPath (filename);
  
  std::ofstream ofs (fullPath, std::ios::app);
  if (!ofs.is_open ()) {
      NS_LOG_ERROR ("ResultWriter: Cannot open " << fullPath);
      return;
  }
  
  ofs << content;
  ofs.close ();
}

} // namespace ns3
