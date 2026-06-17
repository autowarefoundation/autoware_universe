/**
 * Steady-clock timing helpers for simulation loops (min / max / mean / mode).
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace mppi
{
namespace timing
{

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

inline double elapsedMs(const TimePoint& t0, const TimePoint& t1)
{
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

struct TimingStats
{
  size_t count = 0;
  double min_ms = 0.0;
  double max_ms = 0.0;
  double mean_ms = 0.0;
  double mode_ms = 0.0;
};

inline TimingStats summarizeTimingsMs(const std::vector<double>& samples_ms, const double bin_width_ms = 0.1)
{
  TimingStats stats;
  if (samples_ms.empty())
  {
    return stats;
  }

  stats.count = samples_ms.size();
  stats.min_ms = *std::min_element(samples_ms.begin(), samples_ms.end());
  stats.max_ms = *std::max_element(samples_ms.begin(), samples_ms.end());
  stats.mean_ms = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) / static_cast<double>(stats.count);

  std::unordered_map<long long, size_t> histogram;
  histogram.reserve(samples_ms.size());
  for (const double sample_ms : samples_ms)
  {
    const long long bin = static_cast<long long>(std::llround(sample_ms / bin_width_ms));
    ++histogram[bin];
  }

  long long mode_bin = 0;
  size_t mode_count = 0;
  for (const auto& entry : histogram)
  {
    if (entry.second > mode_count)
    {
      mode_count = entry.second;
      mode_bin = entry.first;
    }
  }
  stats.mode_ms = static_cast<double>(mode_bin) * bin_width_ms;
  return stats;
}

inline void printTimingStats(const char* label, const TimingStats& stats)
{
  if (stats.count == 0)
  {
    std::cout << label << ": no samples\n";
    return;
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << label << " (n=" << stats.count << "): min=" << stats.min_ms << " ms  max=" << stats.max_ms
            << " ms  mean=" << stats.mean_ms << " ms  mode=" << stats.mode_ms << " ms";
  if (stats.mean_ms > 0.0)
  {
    std::cout << "  (~" << (1000.0 / stats.mean_ms) << " Hz)";
  }
  std::cout << "\n";
}

/** Records per-step MPPI / viz / post durations for a simulation loop. */
class StepTimingCollector
{
public:
  void reserve(const size_t num_steps)
  {
    total_ms_.reserve(num_steps);
    mppi_ms_.reserve(num_steps);
    dump_ms_.reserve(num_steps);
    viz_ms_.reserve(num_steps);
    post_ms_.reserve(num_steps);
  }

  void beginStep()
  {
    step_start_ = SteadyClock::now();
  }

  void endMppi()
  {
    mppi_end_ = SteadyClock::now();
  }

  void endDump()
  {
    dump_end_ = SteadyClock::now();
  }

  void endViz()
  {
    viz_end_ = SteadyClock::now();
  }

  /** Full step including integrate + log. */
  void endStep()
  {
    const TimePoint step_end = SteadyClock::now();
    mppi_ms_.push_back(elapsedMs(step_start_, mppi_end_));
    dump_ms_.push_back(elapsedMs(mppi_end_, dump_end_));
    viz_ms_.push_back(elapsedMs(dump_end_, viz_end_));
    post_ms_.push_back(elapsedMs(viz_end_, step_end));
    total_ms_.push_back(elapsedMs(step_start_, step_end));
  }

  /** Early loop exit (e.g. ESC): no post phase recorded. */
  void endStepEarlyExit()
  {
    mppi_ms_.push_back(elapsedMs(step_start_, mppi_end_));
    dump_ms_.push_back(elapsedMs(mppi_end_, dump_end_));
    viz_ms_.push_back(elapsedMs(dump_end_, viz_end_));
    post_ms_.push_back(0.0);
    total_ms_.push_back(elapsedMs(step_start_, viz_end_));
  }

  void printReport(const char* title = "Simulation step timing") const
  {
    std::cout << "\n--- " << title << " ---\n";
    printTimingStats("Total step", summarizeTimingsMs(total_ms_));
    printTimingStats("  MPPI (ref + computeControl + sampled traj)", summarizeTimingsMs(mppi_ms_));
    printTimingStats("  Rollout dump (prepare + enqueue)", summarizeTimingsMs(dump_ms_));
    printTimingStats("  Viz (draw + video + imshow)", summarizeTimingsMs(viz_ms_));
    printTimingStats("  Post (integrate + log)", summarizeTimingsMs(post_ms_));
    std::cout << std::defaultfloat;
  }

private:
  std::vector<double> total_ms_;
  std::vector<double> mppi_ms_;
  std::vector<double> dump_ms_;
  std::vector<double> viz_ms_;
  std::vector<double> post_ms_;
  TimePoint step_start_{};
  TimePoint mppi_end_{};
  TimePoint dump_end_{};
  TimePoint viz_end_{};
};

}  // namespace timing
}  // namespace mppi
