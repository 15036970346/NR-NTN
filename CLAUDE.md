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

# Random-access comparison (current primary sim): 4-step vs 2-step RA over multi-RO collision scenario
./ns3 run "ntn-ra-compare"                       # default: compare both, 5 ROs, 200 UE/beam
./ns3 run "ntn-ra-compare --rachType=1"          # 2-step only

# Resource-manager / RRM validation harness
./ns3 run resource-manager-validation

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

### Random Access & PRACH (current work area)

`ntn-ra-compare.cc` is the **primary active simulation**. It models contention-based
random access: 7 beams × N UEs split across multiple Random Access Occasions (ROs),
preambles drawn from 1–63, collisions resolved per aggregation window, comparing
4-step (`--rachType=0`) vs 2-step (`--rachType=1`) RA on success rate and latency.

`PrachDetectionModel` (`model/prach-detection-model.{h,cc}`) is a **system-level**
error model — it does *not* do waveform correlation. It maps PRACH SNR → detection
probability (Pd) and false-alarm probability (Pfa), either fixed
(`SetFixedProbabilities`) or from a link-level curve CSV (`LoadCurveCsv`). SNR is
derived from UE position/elevation. Stats are binned by integer-dB SNR for comparison
against theoretical Pd curves; `PrachDetectionStats` tracks misses and false alarms
(false alarms also consume Msg3 grant RBs).

> **Gotcha:** `SatUtPhy` interference is a mock placeholder — do not rely on it for
> SINR. RA SNR comes from the position/elevation + PRACH-detection path instead.

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
| `model/prach-detection-model.{h,cc}` | System-level PRACH Pd/Pfa error model (SNR → detect, not waveform) |
| `model/resource-manager.{h,cc}` | RRM: RB reservation, EIRP limiting, per-beam allocation (S-band 35 MHz) |
| `examples/ntn-ra-compare.cc` | **Primary sim**: 4-step vs 2-step RA, multi-RO collisions |
| `examples/resource-manager-validation.cc` | RRM validation harness |
| `examples/ntn-system-sim.cc` | Production sim: `--numUes`, `--reuseMode`, `--simTime` |
| `examples/ntn-phase1-sim.cc` | Integrated component test |

## Implementation Constraints

- `GeoBeamScheduler` inherits `NrMacScheduler` but most virtual scheduler SAP methods are stubs — only NTN-specific paths are implemented
- `RohcCompressor` is a simplified reference (RFC 3095 modes implemented structurally, not bit-exact)
- `SatChannel` uses Urban LOS only; NLOS and suburban NTN scenarios are not parameterized
- Fixed 7-beam hexagonal constellation; no dynamic beam steering

## Merging collaborator drops — avoid silent code loss (合并 SOP)

This module is developed across branches (`xys` / `qyh`) and collaborators often hand
over **ZIP snapshots**, not git branches. Every past "merge" that overwrote files and
committed on a **single parent** has eaten code. Treat merging as a first-class,
test-gated procedure — not a file copy.

> **What actually happened (`50f5ad8a7`, 2026-05-29):** the qyh integration was a
> single-parent manual overwrite. `geo-beam-scheduler.cc` was taken wholesale from qyh,
> then our PRACH work was re-injected — but only the **public interface**
> (`Enable/Set/Reset/Get` + `#include` + the `m_prachDetectionModel` member), **not the
> internal call sites** (`DetectActivePreamble`/`RecordActiveGroup` + the per-UE-SNR
> plumbing inside `ProcessPrachWindow`). Result: detection compiled and configured but
> never ran (`ActiveGrp=0`). The accompanying RA success-rate drop was then rationalized
> as "qyh's stricter resource gating, not a bug" — so the regression went unnoticed.
> Restored 2026-06-02 by cherry-picking the detection logic back from `c663d307a`.

### The three failure modes (each needs its own guard)

1. **No shared history** → forced into manual file overwrite → silent loss.
2. **Whole-function-from-one-side** → "interface kept, body dropped" survives compilation.
3. **Regression rationalized** → a metric drop is explained away instead of investigated.

### SOP

1. **Never `unzip` over the working tree and commit on one line.** Import each
   collaborator's drop onto a dedicated vendor branch so the *next* merge has a real
   common ancestor:
   ```bash
   git checkout -b vendor/qyh        # one stable branch per collaborator, reused each drop
   # replace contrib/geo-sat with the zip contents, then:
   git add -A && git commit -m "vendor: qyh <date> drop"
   git checkout <work-branch>
   git merge vendor/qyh              # real two-parent merge → conflicts surface
   ```
2. **Use real `git merge` (two parents), never a single-parent overwrite commit.**
   Conflict markers force you to confront every overlapping change; both lineages stay in
   history for `git diff A...B` and `bisect`.
3. **When both sides changed the same function, resolve per-hunk.** Do **not** "take
   theirs" for the whole function/file — diff the **function body**, not just whether the
   public API still compiles. This is the trap that ate the PRACH detection.
4. **Post-merge audit with codegraph + difftastic** — list what a branch had that the
   merge result lacks (this is cheap and exactly how the loss was found):
   `codegraph_search` / `codegraph_callers` on each "our feature" symbol; confirm the
   **call sites**, not just the definitions, survived. Then **use `difft` to diff the
   function bodies** of every file both sides touched — `git dft <branch>..HEAD -- <file>`
   (or `git dft-show <merge-commit>`) — its syntax-aware output makes the "interface kept,
   body dropped" trap visible in a way line diffs hide. (`difft` = difftastic; git aliases
   `dft`/`dft-show`/`dft-diff` are configured globally.)
5. **Run behavioral smoke checks before AND after the merge.** A feature going silent must
   fail loudly. Minimum bar for RA: `ntn-ra-compare --rachType=0` with detection on must
   show `Missed>0`, `ActiveGrp>0`, empirical Pd≈curve, and a sane success rate; with
   `--enablePrachDetectionErrors=0` it must degrade to all-zeros. Keep one golden
   assertion per core feature (missed-detection, false-alarm Pfa, Msg4 BLER, resource-gated
   success rate).

### Using AI for merges

AI + `git merge` beats AI + manual overwrite — but the win is git surfacing conflicts and
keeping history, **not** AI judgment (an AI session is what ate the code last time). Use AI
to *explain* each conflict hunk and to run the step-4 audit; never let it pick a whole side
unverified, and never let it label a metric regression "expected" without a test or data
backing it.
