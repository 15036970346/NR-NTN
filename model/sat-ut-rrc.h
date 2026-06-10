/*
 * contrib/geo-sat/model/sat-ut-rrc.h
 * UE RRC layer: measurement control + state machine (IDLE/CONNECTED/INACTIVE)
 * State machine modeled on LteUeRrc (src/lte/model/lte-ue-rrc.h:98-114)
 */
#ifndef SAT_UT_RRC_H
#define SAT_UT_RRC_H

#include "ns3/object.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/mobility-model.h"
#include "ns3/callback.h"
#include "ns3/traced-callback.h"
#include "ns3/sat-mac-common.h"
#include <map>

namespace ns3 {

class SatUtRrc : public Object {
public:
    static TypeId GetTypeId (void);
    SatUtRrc ();
    virtual ~SatUtRrc ();

    // ==================== RRC State Machine ====================
    // Modeled on LteUeRrc::State (src/lte/model/lte-ue-rrc.h:98-114)
    enum State {
        IDLE_START = 0,
        IDLE_CELL_SEARCH,
        IDLE_CAMPED_NORMALLY,
        IDLE_RANDOM_ACCESS,
        CONNECTED_NORMALLY,
        CONNECTED_HANDOVER,
        CONNECTED_PHY_PROBLEM,
        RRC_INACTIVE,           // 5G NR RRC_INACTIVE state
        NUM_STATES
    };

    // Get current state
    State GetState () const;

    // State transition (public for external triggers like RA completion)
    void SwitchToState (State newState);

    // ==================== Measurement Control ====================
    // 1. Core: receive raw PHY measurements
    void ProcessRawMeasurement (uint32_t beamId, double rawRsrp);
    void ProcessRawMeasurement (const ServiceObjectId& object, double rawRsrp);

    // 2. Config: set measurement parameters (from SIB or RRC reconfig)
    void SetMeasConfig (MeasConfig config);
    MeasConfig GetMeasConfig () const;

    // 3. Mobility: bind mobility model for position reporting
    void SetMobilityModel (Ptr<MobilityModel> mobility);

    // 4. Identity
    void SetUeId (uint16_t ueId);

    // 5. Report callback
    typedef Callback<void, MeasReport> ReportCallback;
    void SetReportCallback (ReportCallback cb);

    // 6. Service object state
    void SetServingObject (const ServiceObjectId& servingObject);
    ServiceObjectId GetServingObject () const;
    void SetGroundAvailability (bool available);
    void SetSatelliteAvailability (bool available);
    bool IsGroundAvailable () const;
    bool IsSatelliteAvailable () const;
    Time GetServingDwellTime () const;
    bool HasSatisfiedMinDwellTime () const;
    ServiceObjectId GetBestNeighborObject () const;
    double GetServingFilteredMetric () const;
    double GetBestNeighborFilteredMetric () const;

    // ==================== Inactivity Timer ====================
    // Set inactivity timer duration (T3xx series)
    void SetInactivityTimer (Time duration);
    // Reset inactivity timer (called on data activity)
    void ResetInactivityTimer ();

    // ==================== Paging ====================
    // Receive paging message (transitions INACTIVE -> CONNECTED)
    void ReceivePagingMessage ();
    // Get access recovery delay (time from paging to CONNECTED)
    Time GetLastAccessRecoveryDelay () const;

    // ==================== State Transition Trace ====================
    // (ueId, oldState, newState)
    TracedCallback<uint16_t, State, State> m_stateTransitionTrace;

    // Static helper: state name string
    static std::string StateToString (State s);

private:
    // Measurement event evaluation
    void EvaluateMeasurementEvents ();
    void TriggerMeasurementReport ();
    void UpdateNeighborMeasurement (const ServiceObjectId& object, double rawRsrp);
    ServiceObjectId FindBestNeighborObject () const;
    double GetFilteredMetric (const ServiceObjectId& object) const;
    bool IsObjectAvailable (const ServiceObjectId& object) const;

    // Inactivity timer callback
    void OnInactivityTimeout ();

    Ptr<MobilityModel> m_mobility;
    MeasConfig m_measConfig;
    ReportCallback m_reportCb;

    uint16_t m_ueId;
    uint32_t m_currentBeamId;
    double m_filteredRsrp;
    bool m_isConditionMet;
    EventId m_tttEvent;

    ServiceObjectId m_servingObject;
    ServiceObjectId m_tttCandidateObject;
    ServiceObjectId m_lastReportedNeighborObject;

    // Handover-in-progress guard: once a measurement report is sent, suppress
    // further reports until the RRC reconfiguration is applied (SetServingObject)
    // or a guard timeout (modeled on T304) elapses. Prevents duplicate handovers
    // during the report->reconfiguration propagation window.
    bool m_handoverPending {false};
    EventId m_handoverGuardEvent;
    Time m_handoverGuardTimeout {MilliSeconds (1500)};
    std::map<ServiceObjectId, NeighborMeasurement> m_neighborMeasurements;
    Time m_servingSince;
    Time m_tttStartTime;
    bool m_groundAvailable;
    bool m_satelliteAvailable;
    double m_lastA3Delta;

    // State machine
    State m_state;

    // Inactivity timer
    Time m_inactivityDuration;
    EventId m_inactivityTimer;

    // Paging / recovery
    Time m_pagingReceivedTime;
    Time m_lastAccessRecoveryDelay;
};

} // namespace ns3
#endif /* SAT_UT_RRC_H */
