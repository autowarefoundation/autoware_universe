#pragma once
/**
 * Created by Bogdan on 11/01/2023
 */

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace mppi
{
namespace util
{
enum class LOG_LEVEL : int
{
  DEBUG = 0,
  INFO,
  WARNING,
  ERROR,
  NONE
};

static LOG_LEVEL GLOBAL_LOG_LEVEL = LOG_LEVEL::WARNING;

const char BLACK[] = "\033[0;30m";
const char RED[] = "\033[0;31m";
const char GREEN[] = "\033[0;32m";
const char YELLOW[] = "\033[0;33m";
const char BLUE[] = "\033[0;34m";
const char MAGENTA[] = "\033[0;35m";
const char CYAN[] = "\033[0;36m";
const char WHITE[] = "\033[0;37m";
const char RESET[] = "\033[0m";

class MPPILogger
{
public:
  MPPILogger() = default;
  MPPILogger(const MPPILogger& other) = default;
  MPPILogger(MPPILogger&& other) = default;
  virtual ~MPPILogger() = default;
  MPPILogger& operator=(const MPPILogger& other) = default;
  MPPILogger& operator=(MPPILogger&& other) = default;

  explicit MPPILogger(LOG_LEVEL level)
  {
    setLogLevel(level);
  }

  /**
   * @brief Set the Log Level
   *
   * @param level
   */
  void setLogLevel(const LOG_LEVEL& level)
  {
    log_level_ = level;
  }

  /**
   * @brief Set the Output Stream
   *
   * @param output file stream to write (stdout, stderr, nullptr, etc.)
   */
  void setOutputStream(std::FILE* const output)
  {
    output_stream_ = output;
  }

  /**
   * @brief Get the Log Level object
   *
   * @return LOG_LEVEL
   */
  LOG_LEVEL getLogLevel() const
  {
    return log_level_;
  }

  /**
   * @brief Get the Output Stream object
   *
   * @return std::FILE*
   */
  std::FILE* getOutputStream() const
  {
    return output_stream_;
  }

  /**
   * @brief Log debug messages using virtual debug_impl() method
   *
   * @tparam ...Args  variadic template type of args used in the format string
   * @param fmt       format string used in printf
   * @param args      extra args used by the format string fmt
   */
  template <typename... Args>
  void debug(const char* fmt, Args const&... args)
  {
    std::string message = format_string(fmt, args...);
    this->debug_impl(message);
  }

  /**
   * @brief Log info messages using virtual info_impl() method
   *
   * @tparam ...Args  variadic template type of args used in the format string
   * @param fmt       format string used in printf
   * @param args      extra args used by the format string fmt
   */
  template <typename... Args>
  void info(const char* fmt, Args const&... args)
  {
    std::string message = format_string(fmt, args...);
    this->info_impl(message);
  }

  /**
   * @brief Log warning messages using virtual warning_impl() method
   *
   * @tparam ...Args  variadic template type of args used in the format string
   * @param fmt       format string used in printf
   * @param args      extra args used by the format string fmt
   */
  template <typename... Args>
  void warning(const char* fmt, Args const&... args)
  {
    std::string message = format_string(fmt, args...);
    this->warning_impl(message);
  }

  /**
   * @brief Log errror messages using virtual errror_impl() method
   *
   * @tparam ...Args  variadic template type of args used in the format string
   * @param fmt       format string used in printf
   * @param args      extra args used by the format string fmt
   */
  template <typename... Args>
  void error(const char* fmt, Args const&... args)
  {
    std::string message = format_string(fmt, args...);
    this->error_impl(message);
  }

protected:
  LOG_LEVEL log_level_ = GLOBAL_LOG_LEVEL;
  std::FILE* output_stream_ = stdout;

  virtual void debug_impl(const std::string& message)
  {
    if (log_level_ <= LOG_LEVEL::DEBUG)
    {
      surround_fprintf(output_stream_, GREEN, RESET, message);
    }
  }

  virtual void info_impl(const std::string& message)
  {
    if (log_level_ <= LOG_LEVEL::INFO)
    {
      surround_fprintf(output_stream_, CYAN, RESET, message);
    }
  }

  virtual void warning_impl(const std::string& message)
  {
    if (log_level_ <= LOG_LEVEL::WARNING)
    {
      surround_fprintf(output_stream_, YELLOW, RESET, message);
    }
  }

  virtual void error_impl(const std::string& message)
  {
    if (log_level_ <= LOG_LEVEL::ERROR)
    {
      surround_fprintf(output_stream_, RED, RESET, message);
    }
  }

  /**
   * @brief Print message to stream with coloring defined by prefix
   *
   * @param fstream  where the message will be printed to
   * @param prefix   prefix string to print before message. Expected to be a color code
   * @param suffix   suffix string to print after message. Expected to be a color reset code
   * @param message  actual message to be printed
   */
  virtual void surround_fprintf(std::FILE* fstream, const char* prefix, const char* suffix, const std::string& message)
  {
    std::fprintf(fstream, "%s%s%s", prefix, message.c_str(), suffix);
  }

  /**
   * @brief create a string out of format string and variable number of additional arguments
   *
   * @tparam ...Args  variadic template type for extra arguments passed to format_string()
   * @param fmt       format string defining how to display additional arguments
   * @param args      additional arguments for formatting
   *
   * @return          std::string containing formatted text
   */
  template <typename... Args>
  std::string format_string(const char* fmt, Args const&... args)
  {
    // figure out size of formatted string
    std::vector<char> buf(1 + std::snprintf(nullptr, 0, fmt, args...));
    // Fill buffer with formatted string
    std::snprintf(buf.data(), buf.size(), fmt, args...);
    return std::string(buf.data());
  }
};

using MPPILoggerPtr = std::shared_ptr<MPPILogger>;
}  // namespace util
}  // namespace mppi
