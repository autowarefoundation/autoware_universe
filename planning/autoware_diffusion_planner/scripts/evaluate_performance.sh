#!/bin/bash
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Evaluate diffusion planner TRT inference performance.
# Compares legacy (per-frame binding) vs optimized (bind-once + CUDA Graph).
#
# Prerequisites:
#   - TRT engine file (built by running the planner node, or via trtexec)
#   - CUDA and TensorRT development libraries installed
#
# Usage:
#   ./scripts/evaluate_performance.sh --engine /path/to/engine.engine
#   ./scripts/evaluate_performance.sh  # auto-detects engine in default model dir

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCHMARK_SRC="${SCRIPT_DIR}/benchmark_engine.cpp"
BENCHMARK_BIN="${SCRIPT_DIR}/benchmark_engine"

# Defaults
ENGINE_PATH=""
RUNS=300
WARMUP=50
FULL_PIPELINE=""

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
    --engine)
        ENGINE_PATH="$2"
        shift 2
        ;;
    --runs)
        RUNS="$2"
        shift 2
        ;;
    --warmup)
        WARMUP="$2"
        shift 2
        ;;
    --full-pipeline)
        FULL_PIPELINE="--full-pipeline"
        shift
        ;;
    --help)
        echo "Usage: $0 [options]"
        echo ""
        echo "Options:"
        echo "  --engine PATH     TRT engine file (auto-detected if omitted)"
        echo "  --runs N          Benchmark iterations (default: 300)"
        echo "  --warmup N        Warmup iterations (default: 50)"
        echo "  --full-pipeline   Include H2D transfers in timing loop"
        echo ""
        echo "This script compiles a standalone benchmark tool and runs it in"
        echo "both 'legacy' and 'optimized' modes using the same engine file,"
        echo "then prints a side-by-side comparison."
        echo ""
        echo "To build the engine, either:"
        echo "  1. Run the planner node once (engine is cached automatically), or"
        echo "  2. Use trtexec:"
        echo "     trtexec --onnx=model.onnx --plugins=plugins.so --saveEngine=model.engine"
        exit 0
        ;;
    *)
        echo "Unknown option: $1"
        exit 1
        ;;
    esac
done

# Auto-detect engine file
if [[ -z ${ENGINE_PATH} ]]; then
    MODEL_DIR="${HOME}/autoware_data/diffusion_planner/v3.0"
    ENGINE_PATH=$(find "${MODEL_DIR}" -name "*.engine" -print -quit 2>/dev/null || true)
    if [[ -z ${ENGINE_PATH} ]]; then
        echo "ERROR: No engine file found in ${MODEL_DIR}"
        echo ""
        echo "Build the engine first by either:"
        echo "  1. Running the planner node (engine is cached on first inference)"
        echo "  2. Using trtexec:"
        echo "     trtexec --onnx=${MODEL_DIR}/diffusion_planner_simplified.onnx \\"
        echo "       --saveEngine=${MODEL_DIR}/diffusion_planner_simplified.engine"
        exit 1
    fi
fi

if [[ ! -f ${ENGINE_PATH} ]]; then
    echo "ERROR: Engine file not found: ${ENGINE_PATH}"
    exit 1
fi

echo "============================================================"
echo "  Diffusion Planner Performance Evaluation"
echo "============================================================"
echo "  Engine:  ${ENGINE_PATH}"
echo "  Runs:    ${RUNS}"
echo "  Warmup:  ${WARMUP}"
echo "  GPU:     $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'unknown')" # cspell:ignore noheader
echo "------------------------------------------------------------"

# Step 1: Compile benchmark (standalone, no colcon needed)
echo ""
echo "[1/3] Compiling benchmark tool..."

if [[ ! -f ${BENCHMARK_SRC} ]]; then
    echo "ERROR: benchmark_engine.cpp not found at ${BENCHMARK_SRC}"
    exit 1
fi

# Find CUDA include/lib paths (look for cuda_runtime_api.h)
CUDA_INC=""
CUDA_LIB=""
for d in "${CUDA_HOME:-}" /usr/local/cuda /usr/local/cuda-12 /usr/local/cuda-12.*; do
    if [[ -f "${d}/include/cuda_runtime_api.h" ]]; then
        CUDA_INC="${d}/include"
        CUDA_LIB="${d}/lib64"
        break
    fi
done
if [[ -z ${CUDA_INC} ]]; then
    echo "ERROR: Cannot find cuda_runtime_api.h. Set CUDA_HOME."
    exit 1
fi

# cspell:ignore lnvinfer lcudart
g++ -O2 -std=c++17 "-I${CUDA_INC}" "${BENCHMARK_SRC}" -lnvinfer "-L${CUDA_LIB}" -lcudart \
    -o "${BENCHMARK_BIN}"
echo "  Binary: ${BENCHMARK_BIN}"

# Step 2: Run legacy mode
echo ""
echo "[2/3] Running benchmark [legacy mode]..."
echo "  (per-frame shape/address binding, sync D2H, pageable memory)"
echo ""
# shellcheck disable=SC2086
"${BENCHMARK_BIN}" \
    --engine "${ENGINE_PATH}" \
    --mode legacy \
    --runs "${RUNS}" \
    --warmup "${WARMUP}" ${FULL_PIPELINE} 2>&1 | grep -v '^\[TRT\]'

# Step 3: Run optimized mode
echo ""
echo "[3/3] Running benchmark [optimized mode]..."
echo "  (bind-once, CUDA Graph replay, async D2H, pinned memory)"
echo ""
# shellcheck disable=SC2086
"${BENCHMARK_BIN}" \
    --engine "${ENGINE_PATH}" \
    --mode optimized \
    --runs "${RUNS}" \
    --warmup "${WARMUP}" ${FULL_PIPELINE} 2>&1 | grep -v '^\[TRT\]'

# Cleanup compiled binary
rm -f "${BENCHMARK_BIN}"

echo ""
echo "============================================================"
echo "  Evaluation Complete"
echo "  Compare the RESULTS sections above for:"
echo "    - Latency:  Mean, Median, P95, P99"
echo "    - Accuracy: NaN=0, Inf=0 in output check"
echo "    - Memory:   GPU memory usage"
echo "============================================================"
