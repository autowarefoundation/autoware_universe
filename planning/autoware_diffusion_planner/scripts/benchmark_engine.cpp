// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Self-contained TensorRT engine benchmark tool.
// Compares legacy (per-frame binding) vs optimized (bind-once + CUDA Graph) inference.
//
// Build (standalone, no Autoware dependencies):
//   g++ -O2 -std=c++17 benchmark_engine.cpp -lnvinfer -lcudart -o benchmark_engine  // NOLINT
// cspell:ignore lnvinfer lcudart
//
// Usage:
//   ./benchmark_engine --engine model.engine --mode legacy --runs 300
//   ./benchmark_engine --engine model.engine --mode optimized --runs 300
//   ./benchmark_engine --engine model.engine --mode legacy --full-pipeline

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

inline void check_cuda_call(cudaError_t status, const char * file, int line)
{
  if (status != cudaSuccess) {
    std::cerr << "CUDA error: " << cudaGetErrorString(status) << " at " << file << ":" << line
              << "\n";
    std::exit(1);
  }
}
#define CHECK_CUDA(call) check_cuda_call((call), __FILE__, __LINE__)  // NOLINT

class TrtLogger : public nvinfer1::ILogger
{
public:
  Severity min_severity{Severity::kWARNING};

  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= min_severity) {
      std::cerr << "[TRT] " << msg << "\n";
    }
  }
};

struct TensorInfo
{
  std::string name;
  nvinfer1::Dims shape;
  nvinfer1::TensorIOMode io_mode;
  nvinfer1::DataType dtype;
  size_t size_bytes{0};
  void * d_ptr{nullptr};
  void * h_ptr{nullptr};
  bool is_pinned{false};

  [[nodiscard]] bool is_input() const { return io_mode == nvinfer1::TensorIOMode::kINPUT; }
  [[nodiscard]] bool is_output() const { return io_mode == nvinfer1::TensorIOMode::kOUTPUT; }
};

struct BenchmarkConfig
{
  std::string engine_path;
  std::string mode = "optimized";
  int warmup = 50;
  int runs = 300;
  bool full_pipeline = false;
};

// Encapsulates runtime state to reduce function argument counts.
struct BenchmarkContext
{
  nvinfer1::IExecutionContext * ctx;
  std::vector<TensorInfo> & tensors;
  cudaStream_t stream;
  TrtLogger & logger;
  const BenchmarkConfig & config;
};

size_t dtype_size(nvinfer1::DataType dt)
{
  switch (dt) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kBOOL:
      return 1;
    default:
      return 4;
  }
}

size_t dims_volume(const nvinfer1::Dims & dims)
{
  size_t vol = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    vol *= static_cast<size_t>(dims.d[i]);
  }
  return vol;
}

std::string dims_str(const nvinfer1::Dims & dims)
{
  std::string s = "(";
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i > 0) {
      s += ", ";
    }
    s += std::to_string(dims.d[i]);
  }
  return s + ")";
}

std::vector<char> load_engine_file(const std::string & path)
{
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    std::cerr << "ERROR: Cannot open engine file: " << path << "\n";
    std::exit(1);
  }
  auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  file.read(data.data(), size);
  return data;
}

std::vector<TensorInfo> discover_tensors(nvinfer1::ICudaEngine * engine)
{
  std::vector<TensorInfo> tensors;
  const int n = engine->getNbIOTensors();
  for (int i = 0; i < n; ++i) {
    TensorInfo info;
    info.name = engine->getIOTensorName(i);
    info.io_mode = engine->getTensorIOMode(info.name.c_str());
    info.shape = engine->getTensorShape(info.name.c_str());
    info.dtype = engine->getTensorDataType(info.name.c_str());
    for (int d = 0; d < info.shape.nbDims; ++d) {
      if (info.shape.d[d] == -1) {
        info.shape.d[d] = 1;
      }
    }
    info.size_bytes = dims_volume(info.shape) * dtype_size(info.dtype);
    tensors.push_back(info);
  }
  return tensors;
}

void allocate_single_buffer(TensorInfo & t, bool should_pin)
{
  CHECK_CUDA(cudaMalloc(&t.d_ptr, t.size_bytes));
  CHECK_CUDA(cudaMemset(t.d_ptr, 0, t.size_bytes));
  if (should_pin) {
    CHECK_CUDA(cudaHostAlloc(&t.h_ptr, t.size_bytes, cudaHostAllocDefault));
    t.is_pinned = true;
  } else {
    t.h_ptr = std::malloc(t.size_bytes);  // NOLINT
  }
}

void allocate_buffers(std::vector<TensorInfo> & tensors, bool pin_outputs, bool pin_inputs)
{
  for (auto & t : tensors) {
    const bool should_pin = (pin_outputs && t.is_output()) || (pin_inputs && t.is_input());
    allocate_single_buffer(t, should_pin);
  }
}

void free_single_buffer(TensorInfo & t)
{
  if (t.d_ptr) {
    cudaFree(t.d_ptr);
    t.d_ptr = nullptr;
  }
  if (!t.h_ptr) {
    return;
  }
  if (t.is_pinned) {
    cudaFreeHost(t.h_ptr);
  } else {
    std::free(t.h_ptr);  // NOLINT
  }
  t.h_ptr = nullptr;
}

void free_buffers(std::vector<TensorInfo> & tensors)
{
  for (auto & t : tensors) {
    free_single_buffer(t);
  }
}

void fill_single_input(TensorInfo & t, std::mt19937 & gen)
{
  std::normal_distribution<float> dist(0.0f, 0.5f);
  if (t.dtype == nvinfer1::DataType::kFLOAT) {
    auto * h = static_cast<float *>(t.h_ptr);
    const size_t n = t.size_bytes / sizeof(float);
    for (size_t i = 0; i < n; ++i) {
      h[i] = dist(gen);
    }
  } else {
    std::memset(t.h_ptr, 0, t.size_bytes);
  }
  CHECK_CUDA(cudaMemcpy(t.d_ptr, t.h_ptr, t.size_bytes, cudaMemcpyHostToDevice));
}

void fill_random_inputs(std::vector<TensorInfo> & tensors, std::mt19937 & gen)
{
  for (auto & t : tensors) {
    if (t.is_input()) {
      fill_single_input(t, gen);
    }
  }
}

void bind_all(nvinfer1::IExecutionContext * ctx, const std::vector<TensorInfo> & tensors)
{
  for (const auto & t : tensors) {
    if (t.is_input()) {
      ctx->setInputShape(t.name.c_str(), t.shape);
    }
    ctx->setTensorAddress(t.name.c_str(), t.d_ptr);
  }
}

void sync_h2d_all_inputs(const std::vector<TensorInfo> & tensors)
{
  for (const auto & t : tensors) {
    if (t.is_input()) {
      CHECK_CUDA(cudaMemcpy(t.d_ptr, t.h_ptr, t.size_bytes, cudaMemcpyHostToDevice));
    }
  }
}

void async_h2d_all_inputs(const std::vector<TensorInfo> & tensors, cudaStream_t stream)
{
  for (const auto & t : tensors) {
    if (t.is_input()) {
      CHECK_CUDA(cudaMemcpyAsync(t.d_ptr, t.h_ptr, t.size_bytes, cudaMemcpyHostToDevice, stream));
    }
  }
}

void sync_d2h_all_outputs(const std::vector<TensorInfo> & tensors)
{
  for (const auto & t : tensors) {
    if (t.is_output()) {
      CHECK_CUDA(cudaMemcpy(t.h_ptr, t.d_ptr, t.size_bytes, cudaMemcpyDeviceToHost));
    }
  }
}

void async_d2h_all_outputs(const std::vector<TensorInfo> & tensors, cudaStream_t stream)
{
  for (const auto & t : tensors) {
    if (t.is_output()) {
      CHECK_CUDA(cudaMemcpyAsync(t.h_ptr, t.d_ptr, t.size_bytes, cudaMemcpyDeviceToHost, stream));
    }
  }
}

// Legacy mode: rebind shapes + addresses every frame, sync H2D + D2H with pageable memory.
std::vector<double> run_legacy(BenchmarkContext & bench)
{
  std::vector<double> latencies(bench.config.runs);
  for (int i = 0; i < bench.config.runs; ++i) {
    auto t0 = std::chrono::high_resolution_clock::now();

    if (bench.config.full_pipeline) {
      sync_h2d_all_inputs(bench.tensors);
    }
    bind_all(bench.ctx, bench.tensors);
    bench.ctx->enqueueV3(bench.stream);
    CHECK_CUDA(cudaStreamSynchronize(bench.stream));
    sync_d2h_all_outputs(bench.tensors);

    auto t1 = std::chrono::high_resolution_clock::now();
    latencies[i] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  return latencies;
}

// Try to capture a CUDA Graph from enqueueV3.
// Returns null graph_exec if capture fails (e.g. engine uses auxiliary streams).
cudaGraphExec_t try_capture_graph(BenchmarkContext & bench)
{
  const auto saved_severity = bench.logger.min_severity;
  bench.logger.min_severity = nvinfer1::ILogger::Severity::kINTERNAL_ERROR;

  cudaGraph_t graph = nullptr;
  cudaGraphExec_t graph_exec = nullptr;

  cudaStreamBeginCapture(bench.stream, cudaStreamCaptureModeGlobal);
  bench.ctx->enqueueV3(bench.stream);
  cudaError_t err = cudaStreamEndCapture(bench.stream, &graph);

  if (err != cudaSuccess || !graph) {
    cudaGetLastError();
    bench.logger.min_severity = saved_severity;
    return nullptr;
  }

  err = cudaGraphInstantiate(&graph_exec, graph, 0);
  cudaGraphDestroy(graph);

  if (err != cudaSuccess) {
    cudaGetLastError();
    bench.logger.min_severity = saved_severity;
    return nullptr;
  }
  bench.logger.min_severity = saved_severity;
  return graph_exec;
}

void recover_after_failed_capture(BenchmarkContext & bench)
{
  bench.logger.min_severity = nvinfer1::ILogger::Severity::kINTERNAL_ERROR;
  bench.ctx->enqueueV3(bench.stream);
  CHECK_CUDA(cudaStreamSynchronize(bench.stream));
  bench.logger.min_severity = nvinfer1::ILogger::Severity::kWARNING;
}

// Optimized mode: bind once, CUDA Graph (if supported), async D2H with pinned memory.
std::vector<double> run_optimized(BenchmarkContext & bench)
{
  cudaGraphExec_t graph_exec = try_capture_graph(bench);
  const bool use_graph = (graph_exec != nullptr);

  if (use_graph) {
    std::cout << "  CUDA Graph: captured successfully\n";
  } else {
    std::cout << "  CUDA Graph: not supported for this engine (using bind-once + pinned)\n";
    recover_after_failed_capture(bench);
  }

  std::vector<double> latencies(bench.config.runs);
  for (int i = 0; i < bench.config.runs; ++i) {
    auto t0 = std::chrono::high_resolution_clock::now();

    if (bench.config.full_pipeline) {
      async_h2d_all_inputs(bench.tensors, bench.stream);
    }

    if (use_graph) {
      CHECK_CUDA(cudaGraphLaunch(graph_exec, bench.stream));
    } else {
      bench.ctx->enqueueV3(bench.stream);
    }

    async_d2h_all_outputs(bench.tensors, bench.stream);
    CHECK_CUDA(cudaStreamSynchronize(bench.stream));

    auto t1 = std::chrono::high_resolution_clock::now();
    latencies[i] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  if (graph_exec) {
    CHECK_CUDA(cudaGraphExecDestroy(graph_exec));
  }
  return latencies;
}

void print_statistics(const std::vector<double> & raw, const std::string & mode)
{
  auto sorted = raw;
  std::sort(sorted.begin(), sorted.end());
  const int n = static_cast<int>(sorted.size());
  const double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
  const double mean = sum / n;
  const double median = sorted[n / 2];
  const double p95 = sorted[static_cast<size_t>(n * 0.95)];
  const double p99 = sorted[static_cast<size_t>(n * 0.99)];
  double sq_sum = 0.0;
  for (const auto l : sorted) {
    sq_sum += (l - mean) * (l - mean);
  }
  const double stddev = std::sqrt(sq_sum / n);

  std::cout << "------------------------------------------------------\n";
  std::cout << "  RESULTS [" << mode << "]:\n";
  std::cout << "    Mean:   " << mean << " ms\n";
  std::cout << "    Median: " << median << " ms\n";
  std::cout << "    Min:    " << sorted.front() << " ms\n";
  std::cout << "    Max:    " << sorted.back() << " ms\n";
  std::cout << "    P95:    " << p95 << " ms\n";
  std::cout << "    P99:    " << p99 << " ms\n";
  std::cout << "    Std:    " << stddev << " ms\n";
  std::cout << "------------------------------------------------------\n";
}

struct OutputStats
{
  std::string name;
  size_t count{0};
  int nan_count{0};
  int inf_count{0};
  float vmin{0};
  float vmax{0};
  float vmean{0};
};

OutputStats compute_output_stats(const TensorInfo & t)
{
  OutputStats stats;
  stats.name = t.name;
  const auto * data = static_cast<const float *>(t.h_ptr);
  stats.count = t.size_bytes / sizeof(float);
  stats.vmin = 1e30f;
  stats.vmax = -1e30f;
  float vsum = 0;
  for (size_t i = 0; i < stats.count; ++i) {
    stats.nan_count += std::isnan(data[i]) ? 1 : 0;
    stats.inf_count += std::isinf(data[i]) ? 1 : 0;
    stats.vmin = std::min(stats.vmin, data[i]);
    stats.vmax = std::max(stats.vmax, data[i]);
    vsum += data[i];
  }
  stats.vmean = (stats.count > 0) ? vsum / static_cast<float>(stats.count) : 0.0f;
  return stats;
}

void check_outputs(const std::vector<TensorInfo> & tensors)
{
  for (const auto & t : tensors) {
    if (!t.is_output() || t.dtype != nvinfer1::DataType::kFLOAT) {
      continue;
    }
    const auto s = compute_output_stats(t);
    std::cout << "    " << s.name << ": size=" << s.count << " NaN=" << s.nan_count
              << " Inf=" << s.inf_count << " min=" << s.vmin << " max=" << s.vmax
              << " mean=" << s.vmean << "\n";
  }
}

void print_help()
{
  std::cout << "Usage: benchmark_engine [options]\n"
            << "  --engine PATH     TRT engine file (required)\n"
            << "  --mode MODE       'legacy' or 'optimized' (default: optimized)\n"
            << "  --warmup N        Warmup iterations (default: 50)\n"
            << "  --runs N          Benchmark iterations (default: 300)\n"
            << "  --full-pipeline   Include H2D transfers in timing loop\n"
            << "\nModes:\n"
            << "  legacy:     Re-set shapes/addresses per frame, sync D2H (pageable)\n"
            << "              With --full-pipeline: + sync H2D (cudaMemcpy)\n"
            << "  optimized:  Bind once, CUDA Graph replay, async D2H (pinned)\n"
            << "              With --full-pipeline: + async H2D (cudaMemcpyAsync)\n";
}

// Collect raw key-value pairs and flags from argv into a simple map.
struct RawArgs
{
  std::map<std::string, std::string> kv;
  bool full_pipeline{false};
  bool help{false};
};

RawArgs collect_args(int argc, char * argv[])
{
  RawArgs raw;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") {
      raw.help = true;
    } else if (arg == "--full-pipeline") {
      raw.full_pipeline = true;
    } else if (i + 1 < argc) {
      raw.kv[arg] = argv[++i];
    }
  }
  return raw;
}

BenchmarkConfig parse_args(int argc, char * argv[])
{
  const auto raw = collect_args(argc, argv);
  if (raw.help) {
    print_help();
    std::exit(0);
  }

  BenchmarkConfig config;
  config.full_pipeline = raw.full_pipeline;
  if (raw.kv.count("--engine")) {
    config.engine_path = raw.kv.at("--engine");
  }
  if (raw.kv.count("--mode")) {
    config.mode = raw.kv.at("--mode");
  }
  if (raw.kv.count("--warmup")) {
    config.warmup = std::stoi(raw.kv.at("--warmup"));
  }
  if (raw.kv.count("--runs")) {
    config.runs = std::stoi(raw.kv.at("--runs"));
  }
  return config;
}

bool validate_config(const BenchmarkConfig & config)
{
  if (config.engine_path.empty()) {
    std::cerr << "ERROR: --engine is required\n";
    return false;
  }
  if (config.mode != "legacy" && config.mode != "optimized") {
    std::cerr << "ERROR: --mode must be 'legacy' or 'optimized'\n";
    return false;
  }
  return true;
}

void print_header(const BenchmarkConfig & config)
{
  std::cout << "======================================================\n";
  std::cout << "  Diffusion Planner Engine Benchmark\n";
  std::cout << "======================================================\n";
  std::cout << "  Engine:   " << config.engine_path << "\n";
  std::cout << "  Mode:     " << config.mode << "\n";
  std::cout << "  Pipeline: " << (config.full_pipeline ? "full (H2D+infer+D2H)" : "infer+D2H only")
            << "\n";
  std::cout << "  Warmup:   " << config.warmup << "\n";
  std::cout << "  Runs:     " << config.runs << "\n";
  std::cout << "------------------------------------------------------\n";
}

void print_tensors(const std::vector<TensorInfo> & tensors)
{
  std::cout << "  Tensors (" << tensors.size() << "):\n";
  for (const auto & t : tensors) {
    const char * io = t.is_input() ? "IN " : "OUT";
    std::cout << "    [" << io << "] " << t.name << " " << dims_str(t.shape) << " " << t.size_bytes
              << " bytes\n";
  }
}

void run_warmup(BenchmarkContext & bench)
{
  std::cout << "  Warming up (" << bench.config.warmup << " iterations)..." << std::flush;
  for (int i = 0; i < bench.config.warmup; ++i) {
    if (bench.config.mode == "legacy") {
      bind_all(bench.ctx, bench.tensors);
    }
    bench.ctx->enqueueV3(bench.stream);
    CHECK_CUDA(cudaStreamSynchronize(bench.stream));
  }
  std::cout << " done\n";
}

int main(int argc, char * argv[])
{
  const BenchmarkConfig config = parse_args(argc, argv);
  if (!validate_config(config)) {
    return 1;
  }

  const bool use_pinned = (config.mode == "optimized");
  print_header(config);

  // Load engine
  std::cout << "  Loading engine..." << std::flush;
  auto t_load_start = std::chrono::high_resolution_clock::now();

  TrtLogger logger;
  nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(logger);
  auto engine_data = load_engine_file(config.engine_path);
  nvinfer1::ICudaEngine * engine =
    runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
  if (!engine) {
    std::cerr << "\nERROR: Failed to deserialize engine\n";
    return 1;
  }
  nvinfer1::IExecutionContext * context = engine->createExecutionContext();

  auto t_load_end = std::chrono::high_resolution_clock::now();
  const double load_s = std::chrono::duration<double>(t_load_end - t_load_start).count();
  std::cout << " done (" << load_s << "s)\n";

  // Setup
  auto tensors = discover_tensors(engine);
  print_tensors(tensors);

  const bool pin_inputs = use_pinned && config.full_pipeline;
  allocate_buffers(tensors, use_pinned, pin_inputs);
  std::mt19937 gen(42);
  fill_random_inputs(tensors, gen);

  cudaStream_t stream;
  CHECK_CUDA(cudaStreamCreate(&stream));
  bind_all(context, tensors);

  BenchmarkContext bench{context, tensors, stream, logger, config};
  run_warmup(bench);

  // Record GPU memory
  size_t gpu_free = 0;
  size_t gpu_total = 0;
  cudaMemGetInfo(&gpu_free, &gpu_total);
  const size_t gpu_used_mib = (gpu_total - gpu_free) / (1024 * 1024);

  // Benchmark
  std::cout << "  Benchmarking (" << config.runs << " iterations)..." << std::flush;
  const auto latencies = (config.mode == "legacy") ? run_legacy(bench) : run_optimized(bench);
  std::cout << " done\n";

  // Results
  std::cout << "  Output check:\n";
  check_outputs(tensors);
  print_statistics(latencies, config.mode);
  std::cout << "  Load time:   " << load_s << " s\n";
  std::cout << "  GPU memory:  ~" << gpu_used_mib << " MiB\n";
  std::cout << "======================================================\n";

  // Cleanup
  logger.min_severity = nvinfer1::ILogger::Severity::kINTERNAL_ERROR;
  free_buffers(tensors);
  CHECK_CUDA(cudaStreamDestroy(stream));
  delete context;
  delete engine;
  delete runtime;

  return 0;
}
