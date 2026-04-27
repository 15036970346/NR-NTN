/*
 * contrib/geo-sat/model/sat-ut-rrc.cc
 * UE RRC layer: measurement control + state machine implementation
 */
#include "sat-ut-rrc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatUtRrc");
NS_OBJECT_ENSURE_REGISTERED (SatUtRrc);

TypeId SatUtRrc::GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::SatUtRrc")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatUtRrc> ()
        .AddTraceSource ("StateTransition",
                         "RRC state transition trace (ueId, oldState, newState)",
                         MakeTraceSourceAccessor (&SatUtRrc::m_stateTransitionTrace),
                         "ns3::SatUtRrc::StateTransitionCallback");
    return tid;
}

SatUtRrc::SatUtRrc ()
    : m_mobility (nullptr),
      m_ueId (0),
      m_currentBeamId (0),
      m_filteredRsrp (-140.0),
      m_isConditionMet (false),
      m_state (IDLE_START),
      m_inactivityDuration (Seconds (10)),
      m_lastAccessRecoveryDelay (Seconds (0))
{
    NS_LOG_FUNCTION (this);
    // Default measurement config (normally from SIB)
    m_measConfig.rsrpThreshold = -105.0;
    m_measConfig.hysteresis = 2.0;
    m_measConfig.timeToTrigger = MilliSeconds (200);
    m_measConfig.filterCoeff = 0.5;
}

SatUtRrc::~SatUtRrc () {
    NS_LOG_FUNCTION (this);
}

std::string
SatUtRrc::StateToString (State s)
{
    static const char* names[] = {
        "IDLE_START",
        "IDLE_CELL_SEARCH",
        "IDLE_CAMPED_NORMALLY",
        "IDLE_RANDOM_ACCESS",
        "CONNECTED_NORMALLY",
        "CONNECTED_HANDOVER",
        "CONNECTED_PHY_PROBLEM",
        "RRC_INACTIVE",
        "NUM_STATES"
    };
    return (s < NUM_STATES) ? names[s] : "UNKNOWN";
}

SatUtRrc::State
SatUtRrc::GetState () const
{
    return m_state;
}

void
SatUtRrc::SwitchToState (State newState)
{
    if (newState == m_state) return;

    State oldState = m_state;
    m_state = newState;

    NS_LOG_INFO ("UE " << m_ueId << " RRC state: "
                 << StateToString (oldState) << " -> " << StateToString (newState));

    // Fire state transition trace
    m_stateTransitionTrace (m_ueId, oldState, newState);

    // If entering CONNECTED_NORMALLY and we were paged, record recovery delay
    if (newState == CONNECTED_NORMALLY && m_pagingReceivedTime > Seconds (0))
    {
        m_lastAccessRecoveryDelay = Simulator::Now () - m_pagingReceivedTime;
        NS_LOG_INFO ("UE " << m_ueId << " access recovery delay: "
                     << m_lastAccessRecoveryDelay.GetMilliSeconds () << " ms");
        m_pagingReceivedTime = Seconds (0);
    }

    // Start inactivity timer when entering CONNECTED_NORMALLY
    if (newState == CONNECTED_NORMALLY)
    {
        ResetInactivityTimer ();
    }
    else
    {
        // Cancel inactivity timer if leaving CONNECTED_NORMALLY
        if (m_inactivityTimer.IsRunning ())
        {
            Simulator::Cancel (m_inactivityTimer);
        }
    }
}

// ==================== Measurement Control ====================

void SatUtRrc::SetMeasConfig (MeasConfig config) {
    m_measConfig = config;
}

void SatUtRrc::SetMobilityModel (Ptr<MobilityModel> mobility) {
    m_mobility = mobility;
}

void SatUtRrc::SetUeId (uint16_t ueId) {
    m_ueId = ueId;
}

void SatUtRrc::SetReportCallback (ReportCallback cb) {
    m_reportCb = cb;
}

void SatUtRrc::ProcessRawMeasurement (uint32_t beamId, double rawRsrp) {
    // L3 filtering: Fn = (1 - a) * Fn-1 + a * Mn
    double a = m_measConfig.filterCoeff;
    if (m_filteredRsrp == -140.0) {
        m_filteredRsrp = rawRsrp;
    } else {
        m_filteredRsrp = (1.0 - a) * m_filteredRsrp + a * rawRsrp;
    }

    m_currentBeamId = beamId;

    NS_LOG_DEBUG ("UE " << m_ueId << " | Raw RSRP: " << rawRsrp
                  << " dBm | L3 Filtered RSRP: " << m_filteredRsrp << " dBm");

    EvaluateMeasurementEvents ();
}

void SatUtRrc::EvaluateMeasurementEvents () {
    double enterThreshold = m_measConfig.rsrpThreshold - m_measConfig.hysteresis;
    double leaveThreshold = m_measConfig.rsrpThreshold + m_measConfig.hysteresis;

    if (m_filteredRsrp < enterThreshold) {
        if (!m_isConditionMet) {
            m_isConditionMet = true;
            NS_LOG_INFO ("UE " << m_ueId << " RSRP drops below threshold. Starting TTT timer.");
            m_tttEvent = Simulator::Schedule (m_measConfig.timeToTrigger,
                                              &SatUtRrc::TriggerMeasurementReport,
                                              this);
        }
    } else if (m_filteredRsrp > leaveThreshold) {
        if (m_isConditionMet) {
            m_isConditionMet = false;
            if (m_tttEvent.IsRunning ()) {
                Simulator::Cancel (m_tttEvent);
                NS_LOG_INFO ("UE " << m_ueId << " RSRP recovered. Cancelled TTT timer.");
            }
        }
    }
}

void SatUtRrc::TriggerMeasurementReport () {
    NS_LOG_INFO ("UE " << m_ueId << " Measurement Event Triggered! Generating Report.");

    MeasReport report;
    report.ueId = m_ueId;
    report.bestBeamId = m_currentBeamId;
    report.rsrp = m_filteredRsrp;

    if (m_mobility) {
        report.position = m_mobility->GetPosition ();
        NS_LOG_INFO ("Attached UT Position to Report: X=" << report.position.x
                     << ", Y=" << report.position.y
                     << ", Z=" << report.position.z);
    } else {
        NS_LOG_WARN ("MobilityModel is null. Position not included in report.");
    }

    if (!m_reportCb.IsNull ()) {
        m_reportCb (report);
    }
}

// ==================== Inactivity Timer ====================

void
SatUtRrc::SetInactivityTimer (Time duration)
{
    m_inactivityDuration = duration;
}

void
SatUtRrc::ResetInactivityTimer ()
{
    if (m_inactivityTimer.IsRunning ())
    {
        Simulator::Cancel (m_inactivityTimer);
    }
    if (m_state == CONNECTED_NORMALLY)
    {
        m_inactivityTimer = Simulator::Schedule (m_inactivityDuration,
                                                  &SatUtRrc::OnInactivityTimeout,
                                                  this);
    }
}

void
SatUtRrc::OnInactivityTimeout ()
{
    NS_LOG_INFO ("UE " << m_ueId << " inactivity timer expired ("
                 << m_inactivityDuration.GetSeconds () << "s), transitioning to RRC_INACTIVE");
    SwitchToState (RRC_INACTIVE);
}

// ==================== Paging ====================

void
SatUtRrc::ReceivePagingMessage ()
{
    NS_LOG_INFO ("UE " << m_ueId << " received paging message in state "
                 << StateToString (m_state));

    if (m_state == RRC_INACTIVE || m_state == IDLE_CAMPED_NORMALLY)
    {
        m_pagingReceivedTime = Simulator::Now ();
        // Transition through RA to reconnect
        SwitchToState (IDLE_RANDOM_ACCESS);
        // Actual RA is triggered by upper layers; when RA completes, SwitchToState(CONNECTED_NORMALLY)
    }
}

Time
SatUtRrc::GetLastAccessRecoveryDelay () const
{
    return m_lastAccessRecoveryDelay;
}

} // namespace ns3
