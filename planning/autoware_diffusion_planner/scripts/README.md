# Diffusion Planner TRT Inference Benchmark

A standalone benchmark tool for evaluating TensorRT inference performance of the diffusion planner.
No Autoware/colcon build required — compiles directly with `g++`.

## Prerequisites

- NVIDIA GPU with CUDA drivers
- CUDA Toolkit (development headers): `cuda_runtime_api.h`
- TensorRT (development libraries): `libnvinfer`
- A pre-built TensorRT engine file (`.engine`)
- g++ with C++17 support

## Quick Start

The easiest way to run the benchmark is using the evaluation script:

```bash
# Auto-detect engine in ~/autoware_data/diffusion_planner/v3.0/
./scripts/evaluate_performance.sh

# Specify engine explicitly
./scripts/evaluate_performance.sh --engine /path/to/model.engine

# Include H2D transfers in timing (full inference pipeline)
./scripts/evaluate_performance.sh --engine /path/to/model.engine --full-pipeline
```

This script compiles the benchmark, runs both legacy and optimized modes, and prints results side by side.

## Manual Usage

### Step 1: Compile

```bash
cd planning/autoware_diffusion_planner/scripts

# If CUDA is at the default location:
g++ -O2 -std=c++17 benchmark_engine.cpp -lnvinfer -lcudart -o benchmark_engine

# If CUDA headers are in a non-default location (e.g., /usr/local/cuda-12.4):
g++ -O2 -std=c++17 \
  -I/usr/local/cuda-12.4/include \
  -L/usr/local/cuda-12.4/lib64 \
  benchmark_engine.cpp -lnvinfer -lcudart -o benchmark_engine
```

### Step 2: Run

```bash
ENGINE=/path/to/diffusion_planner_batch1_fp32.engine

# Legacy mode (simulates pre-optimization behavior)
./benchmark_engine --engine $ENGINE --mode legacy --runs 300

# Optimized mode (simulates current optimized behavior)
./benchmark_engine --engine $ENGINE --mode optimized --runs 300

# Full pipeline mode (includes H2D + inference + D2H)
./benchmark_engine --engine $ENGINE --mode legacy --runs 300 --full-pipeline
./benchmark_engine --engine $ENGINE --mode optimized --runs 300 --full-pipeline
```

### Command-Line Options

| Option            | Description                          | Default     |
| ----------------- | ------------------------------------ | ----------- |
| `--engine PATH`   | Path to TRT engine file              | _required_  |
| `--mode MODE`     | `legacy` or `optimized`              | `optimized` |
| `--warmup N`      | Number of warmup iterations          | `50`        |
| `--runs N`        | Number of benchmark iterations       | `300`       |
| `--full-pipeline` | Include H2D transfers in timing loop | disabled    |

## Benchmark Modes

### Legacy Mode (`--mode legacy`)

Simulates the **pre-optimization** host-side inference behavior:

- **Per-frame rebinding**: Sets input shapes and tensor addresses on every iteration
- **Synchronous D2H**: Uses `cudaMemcpy` (blocking) with pageable host memory
- **With `--full-pipeline`**: Adds synchronous `cudaMemcpy` H2D for each input tensor

This represents the overhead pattern of the original code, where the execution context
was reconfigured every frame.

### Optimized Mode (`--mode optimized`)

Simulates the **current optimized** host-side inference behavior:

- **Bind-once**: Sets input shapes and tensor addresses once before the benchmark loop
- **CUDA Graph**: Attempts to capture a CUDA Graph for kernel replay (falls back gracefully if unsupported, e.g., on Blackwell GPUs)
- **Async D2H**: Uses `cudaMemcpyAsync` with pinned host memory (`cudaHostAlloc`)
- **With `--full-pipeline`**: Adds async `cudaMemcpyAsync` H2D with pinned input memory

### What `--full-pipeline` Measures

|              | Without `--full-pipeline`        | With `--full-pipeline`                |
| ------------ | -------------------------------- | ------------------------------------- |
| **Scope**    | Inference kernel + D2H           | H2D + Inference kernel + D2H          |
| **Use case** | Isolate kernel + output transfer | Measure complete inference round-trip |

The H2D transfer size for this model is ~570 KB total (16 input tensors), so the
difference is small. Use `--full-pipeline` when you want to measure the complete
end-to-end inference cost as seen by the planning node.

## Building Engine Files

The benchmark requires a pre-built `.engine` file. You can build one by:

### Option 1: Run the planner node

Launch the diffusion planner node. It automatically builds and caches the engine on first inference:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml ...
```

The engine is saved to `~/autoware_data/diffusion_planner/v3.0/`.

### Option 2: Use trtexec

```bash
trtexec \
  --onnx=~/autoware_data/diffusion_planner/v3.0/diffusion_planner_simplified.onnx \
  --saveEngine=~/autoware_data/diffusion_planner/v3.0/diffusion_planner_simplified_batch1_fp32.engine
```

## Interpreting Results

### Output Example

```text
------------------------------------------------------
  RESULTS [optimized]:
    Mean:   5.04 ms      ← Average latency per inference
    Median: 5.03 ms      ← 50th percentile
    Min:    5.01 ms      ← Best case
    Max:    5.49 ms      ← Worst case
    P95:    5.07 ms      ← 95th percentile (tail latency)
    P99:    5.22 ms      ← 99th percentile (worst tail latency)
    Std:    0.04 ms      ← Standard deviation (lower = more predictable)
------------------------------------------------------
  Load time:   0.38 s    ← Engine deserialization time
  GPU memory:  ~2235 MiB ← Total GPU memory used on device
```

### Key Metrics to Compare

| Metric               | What it tells you                                                   |
| -------------------- | ------------------------------------------------------------------- |
| **Mean / Median**    | Typical inference latency                                           |
| **P95 / P99**        | Tail latency — critical for real-time planning                      |
| **Std**              | Inference jitter — lower is better for deterministic planning loops |
| **GPU memory delta** | Memory savings between old and new engine (not the absolute value)  |

### GPU Memory Note

The "GPU memory" value is the **total device memory usage** reported by `cudaMemGetInfo`,
which includes the display driver, desktop compositor, and all other GPU processes.
Only the **delta between runs** is meaningful — it reflects the difference in TRT execution
context scratch memory.

For this model, the optimization (`setMaxAuxStreams(0)`) reduces scratch memory from
~384 MB to ~27 MB (14x reduction) by forcing single-stream execution instead of
allocating 5 auxiliary streams.

## Reference Results

Measured on NVIDIA RTX PRO 6000 Blackwell, TensorRT 10.8, CUDA 12.4:

### Infer + D2H only (without `--full-pipeline`)

| Metric  | OLD engine (legacy) | NEW engine (optimized) | Delta            |
| ------- | ------------------- | ---------------------- | ---------------- |
| Mean    | 5.24 ms             | 4.99 ms                | -4.7%            |
| Median  | 5.18 ms             | 4.96 ms                | -4.3%            |
| P99     | 6.62 ms             | 5.68 ms                | -14.2%           |
| Std     | 0.31 ms             | 0.12 ms                | 2.5x more stable |
| GPU mem | 2701 MiB            | 2327 MiB               | -374 MiB         |

### Full pipeline (with `--full-pipeline`)

| Metric  | OLD engine (legacy) | NEW engine (optimized) | Delta            |
| ------- | ------------------- | ---------------------- | ---------------- |
| Mean    | 5.30 ms             | 5.04 ms                | -4.9%            |
| Median  | 5.29 ms             | 5.03 ms                | -4.9%            |
| P99     | 5.84 ms             | 5.22 ms                | -10.7%           |
| Std     | 0.09 ms             | 0.04 ms                | 2.1x more stable |
| GPU mem | 2665 MiB            | 2235 MiB               | -430 MiB         |

### Summary of Optimizations

| Optimization              | Effect                                                            |
| ------------------------- | ----------------------------------------------------------------- |
| `setMaxAuxStreams(0)`     | 14x less scratch memory (384 MB → 27 MB), single-stream execution |
| Bind-once                 | Eliminates per-frame shape/address rebinding overhead             |
| Async D2H + pinned memory | Overlaps output transfer with CPU work                            |
| ONNX simplification       | Smaller engine file (82 MB → 66 MB), faster deserialization       |
