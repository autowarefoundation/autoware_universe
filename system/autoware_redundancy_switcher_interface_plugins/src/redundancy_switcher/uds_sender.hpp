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
#ifndef UDS_SENDER_HPP_
#define UDS_SENDER_HPP_

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::redundancy_switcher
{

template <typename T>
class UdsSender
{
public:
  explicit UdsSender(const std::string & path, bool use_nonblocking = false);
  ~UdsSender();

  void send(const T & data);
  bool send(const T & data, int timeout_ms);

private:
  int socketfd_{-1};
  std::string socket_path_;

  bool wait_writable(int timeout_ms);
};

template <typename T>
UdsSender<T>::UdsSender(const std::string & path, bool use_nonblocking) : socket_path_(path)
{
  socketfd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
  if (socketfd_ < 0) throw std::runtime_error("UdsSender: socket() failed");

  if (use_nonblocking) {
    int flags = fcntl(socketfd_, F_GETFL, 0);
    fcntl(socketfd_, F_SETFL, flags | O_NONBLOCK);
  }
}

template <typename T>
UdsSender<T>::~UdsSender()
{
  if (socketfd_ >= 0) close(socketfd_);
}

template <typename T>
bool UdsSender<T>::wait_writable(int timeout_ms)
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(socketfd_, &fds);
  timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
  return select(socketfd_ + 1, nullptr, &fds, nullptr, &tv) > 0;
}

template <typename T>
bool UdsSender<T>::send(const T & data, int timeout_ms)
{
  if (!wait_writable(timeout_ms)) return false;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  const std::vector<uint8_t> payload = nlohmann::json::to_cbor(nlohmann::json(data));
  ssize_t n = sendto(
    socketfd_, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr *>(&addr),
    sizeof(addr));

  if (n >= 0) return true;
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ECONNREFUSED || errno == ENOENT)
    return true;  // receiver not yet ready — silently ignore
  throw std::runtime_error("UdsSender: sendto() failed");
}

template <typename T>
void UdsSender<T>::send(const T & data)
{
  if (!send(data, 100)) throw std::runtime_error("UdsSender: send timed out");
}

}  // namespace autoware::redundancy_switcher
#endif  // UDS_SENDER_HPP_
