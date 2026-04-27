#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

cd "${REPO_ROOT}"

./ns3 configure --enable-examples
cmake --build build --target resource-manager-validation -j4
./ns3 run --no-build resource-manager-validation
