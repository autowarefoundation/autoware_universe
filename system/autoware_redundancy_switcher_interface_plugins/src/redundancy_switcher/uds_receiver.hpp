//  Copyright 2025 The Autoware Contributors
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
#ifndef UDS_RECEIVER_HPP_
#define UDS_RECEIVER_HPP_

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::redundancy_switcher
{

template <typename T>
class UdsReceiver
{
public:
  using CallbackType = std::function<void(const T &)>;

  explicit UdsReceiver(const std::string & path, bool use_nonblocking = false);
  UdsReceiver(const std::string & path, bool use_nonblocking, CallbackType callback);
  ~UdsReceiver();

  bool receive(T & data, int timeout_us = 0);
  void receive();

private:
  int socketfd_{-1};
  std::string socket_path_;
  CallbackType callback_;

  bool wait_readable(int timeout_us);
};

template <typename T>
UdsReceiver<T>::UdsReceiver(const std::string & path, bool use_nonblocking) : socket_path_(path)
{
  socketfd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
  if (socketfd_ < 0) throw std::runtime_error("UdsReceiver: socket() failed");

  unlink(socket_path_.c_str());

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (bind(socketfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    throw std::runtime_error("UdsReceiver: bind() failed");

  if (chmod(socket_path_.c_str(), 0666) < 0)
    throw std::runtime_error("UdsReceiver: chmod() failed");

  if (use_nonblocking) {
    int flags = fcntl(socketfd_, F_GETFL, 0);
    fcntl(socketfd_, F_SETFL, flags | O_NONBLOCK);
  }
}

template <typename T>
UdsReceiver<T>::UdsReceiver(
  const std::string & path, bool use_nonblocking, CallbackType callback)
: UdsReceiver(path, use_nonblocking)
{
  callback_ = std::move(callback);
}

template <typename T>
UdsReceiver<T>::~UdsReceiver()
{
  if (socketfd_ >= 0) close(socketfd_);
  unlink(socket_path_.c_str());
}

template <typename T>
bool UdsReceiver<T>::wait_readable(int timeout_us)
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(socketfd_, &fds);
  timeval tv{0, timeout_us};
  return select(socketfd_ + 1, &fds, nullptr, nullptr, &tv) > 0;
}

template <typename T>
bool UdsReceiver<T>::receive(T & data, int timeout_us)
{
  if (!wait_readable(timeout_us)) return false;

  std::vector<uint8_t> buffer(4096);
  ssize_t n = recv(socketfd_, buffer.data(), buffer.size(), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    throw std::runtime_error("UdsReceiver: recv() failed");
  }

  try {
    data = nlohmann::json::from_cbor(buffer.begin(), buffer.begin() + n).get<T>();
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("UdsReceiver: deserialization failed: ") + e.what());
  }
  return true;
}

template <typename T>
void UdsReceiver<T>::receive()
{
  T data;
  if (receive(data, 100'000) && callback_) callback_(data);
}

}  // namespace autoware::redundancy_switcher
#endif  // UDS_RECEIVER_HPP_
