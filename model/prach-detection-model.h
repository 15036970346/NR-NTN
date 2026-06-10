/*
 * System-level NR PRACH detection error model.
 *
 * This model does not implement waveform correlation. It maps a PRACH SNR to
 * detection and false-alarm probabilities, so link-level curves can be used by
 * the system-level random access procedure.
 */
#ifndef PRACH_DETECTION_MODEL_H
#define PRACH_DETECTION_MODEL_H

#include "ns3/object.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3 {

struct PrachDetectionStats
{
  uint64_t activePreambleGroups {0};
  uint64_t detectedGroups {0};
  uint64_t missedGroups {0};
  uint64_t missedUeAttempts {0};
  uint64_t falseAlarmGroups {0};
  uint64_t falseAlarmMsg3GrantRbs {0};
  uint64_t falseAlarmTrials {0};          // 空 preamble 检测试验总数 (Pfa 分母)
  // 按 SNR (整数 dB) 分箱的实测检测统计, 用于与理论 Pd 曲线对照
  std::map<int, uint64_t> activeGroupsBySnrBin;
  std::map<int, uint64_t> detectedGroupsBySnrBin;
};

class PrachDetectionModel : public Object
{
public:
  static TypeId GetTypeId ();

  PrachDetectionModel ();
  ~PrachDetectionModel () override;

  void SetEnabled (bool enabled);
  bool IsEnabled () const;

  void SetFixedProbabilities (double pd, double pfa);
  bool LoadCurveCsv (const std::string& path);
  void ClearCurve ();

  double GetDetectionProbability (double snrDb) const;
  double GetFalseAlarmProbability (double snrDb) const;

  bool DetectActivePreamble (double snrDb);
  bool DetectFalseAlarm (double snrDb);

  void RecordActiveGroup (uint32_t ueAttempts, double snrDb);
  void RecordDetectedGroup (double snrDb);
  void RecordMissedGroup (uint32_t ueAttempts);
  void RecordFalseAlarmTrial ();
  void RecordFalseAlarmGroup (uint32_t msg3GrantRbs);

  PrachDetectionStats GetStats () const;
  void ResetStats ();

  int64_t AssignStreams (int64_t stream);

private:
  struct CurvePoint
  {
    double snrDb;
    double pd;
    double pfa;
  };

  static double ClampProbability (double value);
  static int SnrBin (double snrDb);
  static bool TryParseCurveLine (const std::string& line, CurvePoint& point);
  double Interpolate (double snrDb, bool detectionProbability) const;
  bool DrawBernoulli (double probability);

  bool m_enabled;
  double m_fixedPd;
  double m_fixedPfa;
  std::vector<CurvePoint> m_curve;
  Ptr<UniformRandomVariable> m_rng;
  PrachDetectionStats m_stats;
};

} // namespace ns3

#endif /* PRACH_DETECTION_MODEL_H */
