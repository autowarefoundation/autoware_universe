/**
 * Async MP4 writer fed by OpenGL framebuffer captures (worker thread owns encoding).
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mppi
{
namespace viz
{

class AsyncFramebufferRecorder
{
public:
  AsyncFramebufferRecorder();
  ~AsyncFramebufferRecorder();

  AsyncFramebufferRecorder(const AsyncFramebufferRecorder&) = delete;
  AsyncFramebufferRecorder& operator=(const AsyncFramebufferRecorder&) = delete;

  /** Starts the encoder thread. Returns false if a recording is already active. */
  bool start(const std::string& output_path, const float fps);
  /** Blocks until queued frames are written and the encoder thread exits. */
  void stop();
  bool isRecording() const;

  /**
   * Enqueue one RGBA8 framebuffer (row 0 = bottom). May block briefly if the queue is full.
   * @param repeat_count Write this frame to the MP4 this many times (for video fps > sim rate).
   */
  void enqueueRgbaFrame(std::vector<std::uint8_t>&& rgba, const int width, const int height, const int repeat_count = 1);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace viz
}  // namespace mppi
