# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## geo-sat — NS-3 GEO Satellite NTN Module

NS-3 contrib module simulating GEO satellite networks with a full 5G-NR-style MAC/PHY stack, multi-beam coverage, and NTN-specific protocol extensions (ROHC, HARQ-IR, GEO-delay-tuned RLC/PDCP).

## Build & Run

```bash
# From ns-3-dev root
./ns3 configure --enable-modules=geo-sat
./ns3 build geo-sat

# Integration test (SSB, admission control, scheduling, ROHC, HARQ)
./ns3 run ntn-phase1-sim

# Production scenario with configurable parameters
./ns3 run "ntn-system-sim --numUes=70 --reuseMode=7 --simTime=10"

# Topology visualization (in examples/)
python3 contrib/geo-sat/examples/generate_topology.py   # → topology.xml (NetAnim)
python3 contrib/geo-sat/examples/visualize_topology.py  # → PNG plot
```

## Architecture

### Layer Stack

```
Application (OnOff/Voice)
    ↓
RLC  ←→  PDCP (RohcCompressor)
    ↓
SatUtMac  ←  GeoBeamScheduler
    |              ↑ MeasReport SAP
SatUtPhy      ResourceManager + AdmitControl
    ↓
SatChannel (3GPP TR 38.811 NTN tables)
    ↓
SatGeoFeederPhy (gateway) / SatGeoUserPhy
```

### Two-Level Scheduler (`model/geo-beam-scheduler.h`)

`GeoBeamScheduler` extends `NrMacScheduler` and combines:
- **WRR (Weighted Round-Robin)** — EMERGENCY and VOICE priority queues
- **IPF (Improved Proportional Fair)** — location-aware DATA scheduling

It owns `ResourceManager` (RB allocation, EIRP limiting) and `AdmitControl` (handover admission, 20% emergency reservation, load balancing).

### GEO Delay Adaptation

All RLC/PDCP timers are tuned for ~600 ms RTT via `NtnConfigHelper` static methods. Key settings: extended T-Reordering, T-StatusProhibit, HARQ RTT, and optional ROHC enable (`helper/ntn-config-helper.h`).

### NTN Channel Model (`model/sat-channel.h`)

Extends `ThreeGppChannelModel`, overriding `GetThreeGppTable()` with embedded elevation-dependent LUTs from 3GPP TR 38.811 (S-band and Ka-band, 10°–90°, Urban LOS). `DoRxPowerCalculation()` computes FSPL + beam gain.

### Handover Flow

1. `SatUtPhy` measures SINR → `SatUtRrc` applies L3 filtering + Event A4
2. `MeasReport` delivered via `SatMacPhySapUser` SAP (`model/sat-mac-phy-sap.h`)
3. `GeoBeamScheduler::ReceiveMeasReport()` → `AdmitControl` evaluates target beam
4. `ExecuteHandover()`: context export/import, RNTI update

### Frequency Reuse (`model/frequency-reuse.h`)

- **7-color**: no co-channel interference, full isolation
- **4-color**: +6 dB reuse gain with inter-beam interference; selectable via `--reuseMode`

### Key Enumerations (all in `model/sat-mac-common.h`)

| Enum | Values |
|------|--------|
| `MultipleAccessMode` | SLOTTED_ALOHA, ESSA, FDMA, TDMA |
| `UtType` | UT_PORTABLE, UT_CONSUMER |
| `TrafficType` | TRAFFIC_VOICE, TRAFFIC_DATA, TRAFFIC_HIGH_CAPACITY |
| `ServicePriority` | EMERGENCY, VOICE, DATA, BEST_EFFORT |
| `AdmitDecision` | OK, REJECTED, REDIRECT, QUEUE |
| `HarqState` | IDLE, WAITING_ACK, RETRANSMITTING, SUCCESS, FAILED |

### Statistics & Output

`SatStatsCollector` integrates with NR traces (PDCP/RLC/MAC) and `FlowMonitor` to collect per-UE peak/average throughput and beam utilization. `ResultWriter` exports to files. Output directory is configurable.

## Key Files

| File | Purpose |
|------|---------|
| `model/geo-beam-scheduler.{h,cc}` | Two-level WRR+IPF scheduler + handover |
| `model/sat-ut-mac.{h,cc}` | Terminal MAC state machine (IDLE→CONNECTED→TX/RX) |
| `model/sat-channel.{h,cc}` | NTN channel with 3GPP TR 38.811 tables |
| `model/harq-manager.{h,cc}` | 8-process HARQ-IR with RV index cycling |
| `model/rohc-compressor.{h,cc}` | Simplified RFC 3095 header compression |
| `model/sat-ut-rrc.{h,cc}` | RRC L3 filtering + A2/A4 event triggering |
| `helper/ntn-config-helper.{h,cc}` | Static helpers for GEO RLC/PDCP timer tuning |
| `examples/ntn-system-sim.cc` | Production sim: `--numUes`, `--reuseMode`, `--simTime` |
| `examples/ntn-phase1-sim.cc` | Integrated component test |

## Implementation Constraints

- `GeoBeamScheduler` inherits `NrMacScheduler` but most virtual scheduler SAP methods are stubs — only NTN-specific paths are implemented
- `RohcCompressor` is a simplified reference (RFC 3095 modes implemented structurally, not bit-exact)
- `SatChannel` uses Urban LOS only; NLOS and suburban NTN scenarios are not parameterized
- Fixed 7-beam hexagonal constellation; no dynamic beam steering
