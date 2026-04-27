#!/usr/bin/env bash
set -euo pipefail

cd /home/cgf/Downloads/ns-3-dev
./ns3 run --no-build "ntn-phase1-sim --mode=dl"
