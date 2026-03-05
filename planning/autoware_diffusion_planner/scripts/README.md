# Diffusion Planner TRT Inference Benchmark

Standalone benchmark comparing legacy vs optimized TensorRT inference.
No Autoware/colcon build required.

## Requirements

- NVIDIA GPU with CUDA + TensorRT dev libraries
- g++ with C++17
- A TensorRT engine file (`.engine`)

## Usage

```bash
# Step 1: Build the engine (if you don't have one yet)
trtexec \
  --onnx=~/autoware_data/diffusion_planner/v3.0/diffusion_planner_simplified.onnx \
  --saveEngine=~/autoware_data/diffusion_planner/v3.0/diffusion_planner_simplified.engine

# Step 2: Run benchmark (auto-detects engine in ~/autoware_data/diffusion_planner/v3.0/)
./scripts/evaluate_performance.sh

# Or specify engine and options explicitly
./scripts/evaluate_performance.sh --engine /path/to/model.engine --full-pipeline
```

The script compiles the C++ benchmark, runs both legacy and optimized modes, and prints results side by side.

### Options

| Option            | Description                         | Default     |
| ----------------- | ----------------------------------- | ----------- |
| `--engine PATH`   | Path to TRT engine file             | auto-detect |
| `--runs N`        | Benchmark iterations                | `300`       |
| `--warmup N`      | Warmup iterations                   | `50`        |
| `--full-pipeline` | Include H2D+D2H transfers in timing | disabled    |

## What It Measures

- **Legacy mode**: Per-frame rebinding, synchronous memcpy, pageable memory (pre-optimization behavior)
- **Optimized mode**: Bind-once, CUDA Graph replay, async memcpy, pinned memory (current behavior)

## Reference Results

Measured on NVIDIA RTX PRO 6000 Blackwell, TensorRT 10.8, CUDA 12.4:

| Metric  | Legacy   | Optimized | Improvement      |
| ------- | -------- | --------- | ---------------- |
| Mean    | 5.30 ms  | 5.04 ms   | -4.9%            |
| P99     | 5.84 ms  | 5.22 ms   | -10.7%           |
| Std     | 0.09 ms  | 0.04 ms   | 2.1x more stable |
| GPU mem | 2665 MiB | 2235 MiB  | -430 MiB         |
