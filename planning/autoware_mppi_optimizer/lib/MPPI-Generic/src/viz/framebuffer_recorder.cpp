#include <mppi/viz/framebuffer_recorder.hpp>

#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

namespace
{
constexpr std::size_t kMaxQueuedFrames = 8U;

struct RgbaFrame
{
  std::vector<std::uint8_t> rgba;
  int width = 0;
  int height = 0;
  int repeat_count = 1;

  RgbaFrame() = default;
  RgbaFrame(std::vector<std::uint8_t>&& rgba_in, const int width_in, const int height_in, const int repeat_count_in = 1)
    : rgba(std::move(rgba_in)), width(width_in), height(height_in), repeat_count(repeat_count_in)
  {
  }
};

cv::Mat rgbaBottomUpToBgr(const RgbaFrame& frame)
{
  cv::Mat bgr(frame.height, frame.width, CV_8UC3);
  for (int y = 0; y < frame.height; ++y)
  {
    const std::uint8_t* src_row = frame.rgba.data() + static_cast<std::size_t>(frame.height - 1 - y) * frame.width * 4U;
    std::uint8_t* dst_row = bgr.ptr<std::uint8_t>(y);
    for (int x = 0; x < frame.width; ++x)
    {
      dst_row[x * 3 + 0] = src_row[x * 4 + 2];
      dst_row[x * 3 + 1] = src_row[x * 4 + 1];
      dst_row[x * 3 + 2] = src_row[x * 4 + 0];
    }
  }
  return bgr;
}
}  // namespace

namespace mppi
{
namespace viz
{

struct AsyncFramebufferRecorder::Impl
{
  std::string output_path;
  float fps = 10.0F;
  bool recording = false;
  bool stop_requested = false;

  std::mutex mutex;
  std::condition_variable cv_nonempty;
  std::condition_variable cv_not_full;
  std::deque<RgbaFrame> queue;
  std::thread worker;
};

AsyncFramebufferRecorder::AsyncFramebufferRecorder() : impl_(new Impl())
{
}

AsyncFramebufferRecorder::~AsyncFramebufferRecorder()
{
  stop();
}

bool AsyncFramebufferRecorder::start(const std::string& output_path, const float fps)
{
  if (impl_->recording)
  {
    return false;
  }

  impl_->output_path = output_path;
  impl_->fps = fps;
  impl_->stop_requested = false;
  impl_->recording = true;

  impl_->worker = std::thread([this]() {
    cv::VideoWriter writer;
    bool writer_open = false;
    std::size_t frames_written = 0U;

    while (true)
    {
      RgbaFrame frame;
      {
        std::unique_lock<std::mutex> lock(impl_->mutex);
        impl_->cv_nonempty.wait(lock, [this]() { return impl_->stop_requested || !impl_->queue.empty(); });
        if (impl_->stop_requested && impl_->queue.empty())
        {
          break;
        }
        frame = std::move(impl_->queue.front());
        impl_->queue.pop_front();
        impl_->cv_not_full.notify_one();
      }

      const cv::Mat bgr = rgbaBottomUpToBgr(frame);
      if (!writer_open)
      {
        writer.open(impl_->output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), static_cast<double>(impl_->fps),
                    cv::Size(frame.width, frame.height));
        writer_open = writer.isOpened();
        if (!writer_open)
        {
          std::cerr << "AsyncFramebufferRecorder: failed to open " << impl_->output_path << "\n";
          std::unique_lock<std::mutex> lock(impl_->mutex);
          impl_->stop_requested = true;
          impl_->queue.clear();
          break;
        }
        std::cout << "Recording video to " << impl_->output_path << " (" << frame.width << "x" << frame.height
                  << " @ " << impl_->fps << " fps)\n";
      }

      const int repeats = std::max(1, frame.repeat_count);
      for (int r = 0; r < repeats; ++r)
      {
        writer.write(bgr);
        ++frames_written;
      }
    }

    if (writer_open)
    {
      std::cout << "Wrote " << frames_written << " video frames to " << impl_->output_path << "\n";
    }
  });

  return true;
}

void AsyncFramebufferRecorder::stop()
{
  if (!impl_->recording)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stop_requested = true;
  }
  impl_->cv_nonempty.notify_one();
  if (impl_->worker.joinable())
  {
    impl_->worker.join();
  }

  impl_->queue.clear();
  impl_->recording = false;
  impl_->stop_requested = false;
}

bool AsyncFramebufferRecorder::isRecording() const
{
  return impl_->recording;
}

void AsyncFramebufferRecorder::enqueueRgbaFrame(std::vector<std::uint8_t>&& rgba, const int width, const int height,
                                                const int repeat_count)
{
  if (!impl_->recording || width <= 0 || height <= 0)
  {
    return;
  }

  const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4U;
  if (rgba.size() != expected)
  {
    return;
  }

  {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->cv_not_full.wait(lock, [this]() { return impl_->stop_requested || impl_->queue.size() < kMaxQueuedFrames; });
    if (impl_->stop_requested)
    {
      return;
    }
    impl_->queue.push_back(RgbaFrame{ std::move(rgba), width, height, std::max(1, repeat_count) });
  }
  impl_->cv_nonempty.notify_one();
}

}  // namespace viz
}  // namespace mppi
