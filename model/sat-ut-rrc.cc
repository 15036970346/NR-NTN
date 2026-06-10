/*
 * contrib/geo-sat/model/sat-ut-rrc.cc
 * UE RRC layer: measurement control + state machine implementation
 */
#include "sat-ut-rrc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

namespace {

std::string
ServiceObjectToString (const ServiceObjectId& object)
{
    switch (object.type)
    {
    case ServiceObjectType::SERVICE_OBJECT_GROUND:
        return "GROUND:" + std::to_string (object.objectId);
    case ServiceObjectType::SERVICE_OBJECT_SAT_BEAM:
        return "SAT_BEAM:" + std::to_string (object.objectId);
    default:
        return "UNKNOWN:0";
    }
}

} // namespace

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
      m_servingSince (Seconds (0)),
      m_tttStartTime (Seconds (0)),
      m_groundAvailable (true),
      m_satelliteAvailable (true),
      m_lastA3Delta (-1e9),
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

MeasConfig
SatUtRrc::GetMeasConfig () const
{
    return m_measConfig;
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
    m_currentBeamId = beamId;
    ProcessRawMeasurement ({ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, beamId}, rawRsrp);
}

void
SatUtRrc::ProcessRawMeasurement (const ServiceObjectId& object, double rawRsrp)
{
    UpdateNeighborMeasurement (object, rawRsrp);
    EvaluateMeasurementEvents ();
}

void
SatUtRrc::SetServingObject (const ServiceObjectId& servingObject)
{
    if (m_servingObject == servingObject)
    {
        return;
    }

    NS_LOG_INFO ("UE " << m_ueId << " serving object updated: "
                 << ServiceObjectToString (m_servingObject) << " -> "
                 << ServiceObjectToString (servingObject));

    m_servingObject = servingObject;
    m_servingSince = Simulator::Now ();
    m_tttCandidateObject = ServiceObjectId ();
    m_lastReportedNeighborObject = ServiceObjectId ();
    m_isConditionMet = false;
    m_lastA3Delta = -1e9;
    if (m_tttEvent.IsRunning ())
    {
        Simulator::Cancel (m_tttEvent);
    }

    // Reconfiguration applied: clear handover-pending guard.
    m_handoverPending = false;
    if (m_handoverGuardEvent.IsRunning ())
    {
        Simulator::Cancel (m_handoverGuardEvent);
    }

    if (servingObject.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
        m_currentBeamId = servingObject.objectId;
    }
}

ServiceObjectId
SatUtRrc::GetServingObject () const
{
    return m_servingObject;
}

void
SatUtRrc::SetGroundAvailability (bool available)
{
    if (m_groundAvailable == available)
    {
        return;
    }

    m_groundAvailable = available;
    NS_LOG_INFO ("UE " << m_ueId << " ground availability set to " << m_groundAvailable);
    EvaluateMeasurementEvents ();
}

void
SatUtRrc::SetSatelliteAvailability (bool available)
{
    if (m_satelliteAvailable == available)
    {
        return;
    }

    m_satelliteAvailable = available;
    NS_LOG_INFO ("UE " << m_ueId << " satellite availability set to " << m_satelliteAvailable);
    EvaluateMeasurementEvents ();
}

bool
SatUtRrc::IsGroundAvailable () const
{
    return m_groundAvailable;
}

bool
SatUtRrc::IsSatelliteAvailable () const
{
    return m_satelliteAvailable;
}

Time
SatUtRrc::GetServingDwellTime () const
{
    if (!m_servingObject.IsValid ())
    {
        return Seconds (0);
    }
    return Simulator::Now () - m_servingSince;
}

bool
SatUtRrc::HasSatisfiedMinDwellTime () const
{
    return GetServingDwellTime () >= m_measConfig.minDwellTime;
}

ServiceObjectId
SatUtRrc::GetBestNeighborObject () const
{
    return FindBestNeighborObject ();
}

double
SatUtRrc::GetServingFilteredMetric () const
{
    return GetFilteredMetric (m_servingObject);
}

double
SatUtRrc::GetBestNeighborFilteredMetric () const
{
    return GetFilteredMetric (FindBestNeighborObject ());
}

void
SatUtRrc::UpdateNeighborMeasurement (const ServiceObjectId& object, double rawRsrp)
{
    if (!object.IsValid ())
    {
        NS_LOG_WARN ("UE " << m_ueId << " received measurement for invalid service object");
        return;
    }

    NeighborMeasurement& meas = m_neighborMeasurements[object];
    meas.object = object;
    meas.rawMetric = rawRsrp;
    meas.sampleTime = Simulator::Now ();
    meas.isAvailable = IsObjectAvailable (object);

    double a = m_measConfig.filterCoeff;
    if (meas.filteredMetric <= -140.0)
    {
        meas.filteredMetric = rawRsrp;
    }
    else
    {
        meas.filteredMetric = (1.0 - a) * meas.filteredMetric + a * rawRsrp;
    }

    if (object.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
        m_currentBeamId = object.objectId;
        m_filteredRsrp = meas.filteredMetric;
    }

    if (!m_servingObject.IsValid ())
    {
        NS_LOG_INFO ("UE " << m_ueId << " has no serving object yet. Adopting first measurement as serving: "
                     << ServiceObjectToString (object));
        m_servingObject = object;
        m_servingSince = Simulator::Now ();
    }

    NS_LOG_DEBUG ("UE " << m_ueId << " measurement update | object="
                  << ServiceObjectToString (object)
                  << " | raw=" << rawRsrp << " dBm"
                  << " | filtered=" << meas.filteredMetric << " dBm"
                  << " | available=" << meas.isAvailable);
}

ServiceObjectId
SatUtRrc::FindBestNeighborObject () const
{
    ServiceObjectId bestObject;
    double bestMetric = -1e9;

    for (const auto& entry : m_neighborMeasurements)
    {
        const NeighborMeasurement& meas = entry.second;
        if (!meas.object.IsValid () || meas.object == m_servingObject)
        {
            continue;
        }

        if (!IsObjectAvailable (meas.object))
        {
            continue;
        }

        if (meas.filteredMetric > bestMetric)
        {
            bestMetric = meas.filteredMetric;
            bestObject = meas.object;
        }
    }

    return bestObject;
}

double
SatUtRrc::GetFilteredMetric (const ServiceObjectId& object) const
{
    auto it = m_neighborMeasurements.find (object);
    if (it == m_neighborMeasurements.end ())
    {
        return -140.0;
    }
    return it->second.filteredMetric;
}

bool
SatUtRrc::IsObjectAvailable (const ServiceObjectId& object) const
{
    if (!object.IsValid ())
    {
        return false;
    }

    switch (object.type)
    {
    case ServiceObjectType::SERVICE_OBJECT_GROUND:
        return m_measConfig.enableGroundService && m_groundAvailable;
    case ServiceObjectType::SERVICE_OBJECT_SAT_BEAM:
        return m_measConfig.enableSatelliteService && m_satelliteAvailable;
    default:
        return false;
    }
}

void SatUtRrc::EvaluateMeasurementEvents () {
    if (!m_servingObject.IsValid ())
    {
        NS_LOG_DEBUG ("UE " << m_ueId << " has no serving object. Skip A3 evaluation.");
        return;
    }

    if (m_handoverPending)
    {
        NS_LOG_DEBUG ("UE " << m_ueId << " handover in progress (awaiting reconfiguration). Skip evaluation.");
        return;
    }

    const double servingMetric = GetFilteredMetric (m_servingObject);
    const bool servingAvailable = IsObjectAvailable (m_servingObject);
    const ServiceObjectId bestNeighbor = FindBestNeighborObject ();
    const double neighborMetric = GetFilteredMetric (bestNeighbor);
    const bool hasNeighbor = bestNeighbor.IsValid ();
    const bool neighborAvailable = hasNeighbor && IsObjectAvailable (bestNeighbor);

    NS_LOG_INFO ("UE " << m_ueId << " A3 evaluation | serving="
                 << ServiceObjectToString (m_servingObject)
                 << " metric=" << servingMetric
                 << " dBm | neighbor=" << ServiceObjectToString (bestNeighbor)
                 << " metric=" << (hasNeighbor ? neighborMetric : -140.0)
                 << " dBm | groundAvailable=" << m_groundAvailable
                 << " | satelliteAvailable=" << m_satelliteAvailable);

    if (!servingAvailable)
    {
        if (m_tttEvent.IsRunning ())
        {
            Simulator::Cancel (m_tttEvent);
            NS_LOG_INFO ("UE " << m_ueId << " serving object became unavailable. Cancelled pending A3 TTT.");
        }
        m_isConditionMet = false;
        m_tttCandidateObject = ServiceObjectId ();

        if (neighborAvailable && bestNeighbor != m_lastReportedNeighborObject)
        {
            NS_LOG_WARN ("UE " << m_ueId
                         << " serving object unavailable. Triggering availability-driven emergency report to "
                         << ServiceObjectToString (bestNeighbor)
                         << " instead of regular A3.");
            m_tttCandidateObject = bestNeighbor;
            TriggerMeasurementReport ();
        }
        else if (!neighborAvailable)
        {
            NS_LOG_WARN ("UE " << m_ueId
                         << " serving object unavailable, but no available neighbor exists for emergency report.");
        }
        return;
    }

    if (!neighborAvailable)
    {
        if (m_tttEvent.IsRunning ())
        {
            Simulator::Cancel (m_tttEvent);
            NS_LOG_INFO ("UE " << m_ueId << " no available neighbor remains. Cancelled A3 TTT.");
        }
        m_isConditionMet = false;
        m_tttCandidateObject = ServiceObjectId ();
        return;
    }

    const double a3Threshold = servingMetric + m_measConfig.a3Offset + m_measConfig.hysteresis;
    const bool a3ConditionMet = neighborMetric >= a3Threshold;
    m_lastA3Delta = neighborMetric - servingMetric;

    NS_LOG_INFO ("UE " << m_ueId << " A3 compare | servingMetric=" << servingMetric
                 << " dBm | neighborMetric=" << neighborMetric
                 << " dBm | delta=" << m_lastA3Delta
                 << " dB | required>=" << (m_measConfig.a3Offset + m_measConfig.hysteresis) << " dB");

    if (a3ConditionMet && !HasSatisfiedMinDwellTime ())
    {
        if (m_tttEvent.IsRunning ())
        {
            Simulator::Cancel (m_tttEvent);
            NS_LOG_INFO ("UE " << m_ueId << " A3 condition met but dwell time insufficient. Cancelled pending TTT.");
        }
        m_isConditionMet = false;
        m_tttCandidateObject = ServiceObjectId ();
        NS_LOG_INFO ("UE " << m_ueId << " A3 suppressed by min dwell time. Current dwell="
                     << GetServingDwellTime ().GetMilliSeconds () << " ms, required="
                     << m_measConfig.minDwellTime.GetMilliSeconds () << " ms");
        return;
    }

    if (a3ConditionMet)
    {
        const bool sameCandidate = (bestNeighbor == m_tttCandidateObject);
        if (!m_tttEvent.IsRunning () || !sameCandidate)
        {
            if (m_tttEvent.IsRunning ())
            {
                Simulator::Cancel (m_tttEvent);
                NS_LOG_INFO ("UE " << m_ueId << " A3 candidate changed to "
                             << ServiceObjectToString (bestNeighbor)
                             << ". Restarting TTT.");
            }

            m_isConditionMet = true;
            m_tttCandidateObject = bestNeighbor;
            m_tttStartTime = Simulator::Now ();
            NS_LOG_INFO ("UE " << m_ueId << " A3 condition satisfied. Starting TTT for candidate "
                         << ServiceObjectToString (bestNeighbor)
                         << " | TTT=" << m_measConfig.timeToTrigger.GetMilliSeconds () << " ms");
            m_tttEvent = Simulator::Schedule (m_measConfig.timeToTrigger,
                                              &SatUtRrc::TriggerMeasurementReport,
                                              this);
        }
        else
        {
            NS_LOG_INFO ("UE " << m_ueId << " A3 condition remains satisfied for candidate "
                         << ServiceObjectToString (bestNeighbor)
                         << ". Waiting for TTT expiry.");
        }
        return;
    }

    if (m_tttEvent.IsRunning ())
    {
        Simulator::Cancel (m_tttEvent);
        NS_LOG_INFO ("UE " << m_ueId << " A3 condition no longer satisfied. Cancelled TTT.");
    }

    m_isConditionMet = false;
    m_tttCandidateObject = ServiceObjectId ();
}

void SatUtRrc::TriggerMeasurementReport () {
    if (!m_servingObject.IsValid ())
    {
        NS_LOG_WARN ("UE " << m_ueId << " cannot generate measurement report without a serving object.");
        return;
    }

    const bool servingAvailable = IsObjectAvailable (m_servingObject);
    const ServiceObjectId currentBestNeighbor =
        m_tttCandidateObject.IsValid () ? m_tttCandidateObject : FindBestNeighborObject ();
    const bool neighborAvailable = currentBestNeighbor.IsValid () && IsObjectAvailable (currentBestNeighbor);
    const double servingMetric = GetFilteredMetric (m_servingObject);
    const double neighborMetric = GetFilteredMetric (currentBestNeighbor);

    if (servingAvailable)
    {
        const double a3Threshold = servingMetric + m_measConfig.a3Offset + m_measConfig.hysteresis;
        const bool stillValid = neighborAvailable &&
                                currentBestNeighbor == FindBestNeighborObject () &&
                                HasSatisfiedMinDwellTime () &&
                                neighborMetric >= a3Threshold;

        NS_LOG_INFO ("UE " << m_ueId << " TTT expired for candidate "
                     << ServiceObjectToString (currentBestNeighbor)
                     << " | servingMetric=" << servingMetric
                     << " dBm | neighborMetric=" << neighborMetric
                     << " dBm | threshold=" << a3Threshold << " dBm");

        if (!stillValid)
        {
            NS_LOG_INFO ("UE " << m_ueId << " TTT expiry check failed. A3 condition is no longer valid.");
            m_isConditionMet = false;
            m_tttCandidateObject = ServiceObjectId ();
            return;
        }
    }
    else
    {
        NS_LOG_WARN ("UE " << m_ueId << " generating emergency availability-driven measurement report. "
                     << "This is not regular A3.");
        if (!neighborAvailable)
        {
            NS_LOG_WARN ("UE " << m_ueId << " emergency report aborted because no available neighbor remains.");
            m_tttCandidateObject = ServiceObjectId ();
            return;
        }
    }

    MeasReport report;
    report.ueId = m_ueId;
    report.eventType = servingAvailable ? MeasEventType::MEAS_EVENT_A3
                                        : MeasEventType::MEAS_EVENT_SERVING_UNAVAILABLE;
    report.measurementTime = Simulator::Now ();

    report.servingObject = m_servingObject;
    report.servingObjectType = m_servingObject.type;
    report.servingObjectId = m_servingObject.objectId;
    report.servingMetric = servingMetric;
    report.servingFilteredMetric = servingMetric;
    report.servingAvailable = servingAvailable;

    report.bestNeighborObject = currentBestNeighbor;
    report.bestNeighborObjectType = currentBestNeighbor.type;
    report.bestNeighborObjectId = currentBestNeighbor.objectId;
    report.bestNeighborMetric = neighborMetric;
    report.bestNeighborFilteredMetric = neighborMetric;
    report.bestNeighborAvailable = neighborAvailable;
    report.a3Delta = neighborMetric - servingMetric;

    report.groundAvailable = m_groundAvailable;
    report.satelliteAvailable = m_satelliteAvailable;

    auto servingIt = m_neighborMeasurements.find (m_servingObject);
    if (servingIt != m_neighborMeasurements.end ())
    {
        report.servingSnapshot = servingIt->second;
    }

    auto neighborIt = m_neighborMeasurements.find (currentBestNeighbor);
    if (neighborIt != m_neighborMeasurements.end ())
    {
        report.bestNeighborSnapshot = neighborIt->second;
    }

    if (currentBestNeighbor.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
        report.bestBeamId = currentBestNeighbor.objectId;
        report.rsrp = neighborMetric;
    }
    else if (m_servingObject.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
        report.bestBeamId = m_servingObject.objectId;
        report.rsrp = servingMetric;
    }
    else
    {
        report.bestBeamId = 0;
        report.rsrp = neighborMetric;
    }

    if (m_mobility) {
        report.position = m_mobility->GetPosition ();
        NS_LOG_INFO ("Attached UT Position to Report: X=" << report.position.x
                     << ", Y=" << report.position.y
                     << ", Z=" << report.position.z);
    } else {
        NS_LOG_WARN ("MobilityModel is null. Position not included in report.");
    }

    NS_LOG_INFO ("UE " << m_ueId << " Measurement Report generated | event="
                 << (report.eventType == MeasEventType::MEAS_EVENT_A3 ? "A3" : "SERVING_UNAVAILABLE")
                 << " | serving=" << ServiceObjectToString (report.servingObject)
                 << " (" << report.servingFilteredMetric << " dBm)"
                 << " | neighbor=" << ServiceObjectToString (report.bestNeighborObject)
                 << " (" << report.bestNeighborFilteredMetric << " dBm)"
                 << " | delta=" << report.a3Delta << " dB");

    m_lastReportedNeighborObject = currentBestNeighbor;

    // Enter handover-pending: suppress further reports until reconfiguration is
    // applied (SetServingObject) or the guard timeout elapses (modeled on T304).
    m_handoverPending = true;
    if (m_handoverGuardEvent.IsRunning ())
    {
        Simulator::Cancel (m_handoverGuardEvent);
    }
    m_handoverGuardEvent = Simulator::Schedule (m_handoverGuardTimeout, [this] () {
        NS_LOG_INFO ("UE " << m_ueId << " handover guard timeout expired; clearing pending flag.");
        m_handoverPending = false;
    });

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
