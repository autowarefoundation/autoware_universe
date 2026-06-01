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
#include "redundancy_switcher_adapter.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <redundancy_switcher_interface/detail/overloaded.hpp>

namespace autoware::redundancy_switcher
{

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

void RedundancySwitcherAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("RedundancySwitcherAdapter: node is null");
  if (!gateway) throw std::invalid_argument("RedundancySwitcherAdapter: gateway is null");
  node_ = node;
  gateway_ = gateway;

  const std::string sender_path = node_->declare_parameter<std::string>("uds.switcher_command_path");
  const std::string receiver_path = node_->declare_parameter<std::string>("uds.switcher_status_path");
  election_status_timeout_milli_ = node_->declare_parameter<double>(
    "uds.election_status_timeout_milli");
  const double election_request_send_interval_milli = node_->declare_parameter<double>(
    "uds.election_request_send_interval_milli");
  is_main_ecu_ = node_->has_parameter("is_main_ecu")
    ? node_->get_parameter("is_main_ecu").as_bool()
    : node_->declare_parameter<bool>("is_main_ecu");

  RCLCPP_INFO(
    node_->get_logger(),
    "RedundancySwitcherAdapter UDS config: send_path=%s, recv_path=%s, send_interval_ms=%.3f, "
    "status_timeout_ms=%.3f, is_main_ecu=%s",
    sender_path.c_str(), receiver_path.c_str(), election_request_send_interval_milli,
    election_status_timeout_milli_, is_main_ecu_ ? "true" : "false");

  uds_sender_ = std::make_unique<UdsSender<ElectionRequest>>(sender_path);
  uds_receiver_ = std::make_unique<UdsReceiver<ElectionStatus>>(
    receiver_path, /*use_nonblocking=*/false,
    [this](const ElectionStatus & s) { on_switcher_status(s); });

  const auto interval_ms = std::chrono::duration<double, std::milli>(election_request_send_interval_milli);
  timer_ = node_->create_wall_timer(
    interval_ms, [this]() {
      send_election_request(ElectionRequest{/*.self_fault_request=*/false, /*.reset=*/false});
      check_election_status_timeout(); });

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  const std::string hardware_id = is_main_ecu_ ? "main_ecu_redundancy_switcher" : "sub_ecu_redundancy_switcher";
  updater_->setHardwareID(hardware_id);
  updater_->add("main_ecu_fault", this, &RedundancySwitcherAdapter::update_main_ecu_fault_diag);
  updater_->add("sub_ecu_fault", this, &RedundancySwitcherAdapter::update_sub_ecu_fault_diag);
  updater_->add("main_vcu_fault", this, &RedundancySwitcherAdapter::update_main_vcu_fault_diag);
  updater_->add("sub_vcu_fault", this, &RedundancySwitcherAdapter::update_sub_vcu_fault_diag);
  updater_->add(
    "main_ecu_to_sub_ecu_link_fault", this,
    &RedundancySwitcherAdapter::update_main_ecu_to_sub_ecu_link_fault_diag);
  updater_->add(
    "main_ecu_to_main_vcu_link_fault", this,
    &RedundancySwitcherAdapter::update_main_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "main_ecu_to_sub_vcu_link_fault", this,
    &RedundancySwitcherAdapter::update_main_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_main_vcu_link_fault", this,
    &RedundancySwitcherAdapter::update_sub_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_sub_vcu_link_fault", this,
    &RedundancySwitcherAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "main_vcu_to_sub_vcu_link_fault", this,
    &RedundancySwitcherAdapter::update_main_vcu_to_sub_vcu_link_fault_diag);

  // Start UDS receiver thread
  is_uds_receiver_running_.store(true, std::memory_order_release);
  uds_receiver_thread_ = std::thread(&RedundancySwitcherAdapter::uds_receive_loop, this);
  RCLCPP_INFO(node_->get_logger(), "UDS receiver thread started");
}

RedundancySwitcherAdapter::~RedundancySwitcherAdapter()
{
  is_uds_receiver_running_.store(false, std::memory_order_release);
  if (uds_receiver_thread_.joinable()) {
    uds_receiver_thread_.join();
  }
  RCLCPP_INFO(node_->get_logger(), "RedundancySwitcherAdapter destroyed");
}

void RedundancySwitcherAdapter::uds_receive_loop()
{
  while (is_uds_receiver_running_.load(std::memory_order_acquire)) {
    try {
      uds_receiver_->receive();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Error in UDS receiver thread: %s", e.what());
    }
  }
}

void RedundancySwitcherAdapter::send_election_request(const ElectionRequest & request)
{
  uds_sender_->send(request);
}

void RedundancySwitcherAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const SelfInterruptionCommand &) {
        send_election_request(ElectionRequest{/*.self_fault_request=*/true, /*.reset=*/false});
      },
      [this](const ResetCommand &) {
        send_election_request(ElectionRequest{/*.self_fault_request=*/false, /*.reset=*/true});
      },
      [this](const UpdateAutowareReadyCommand & cmd) {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        autoware_ready_ = cmd.value;
      },
      [this](const UpdateAnotherEcuAvailabilityTimeoutCommand & cmd) {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        another_ecu_availability_timeout_ = cmd.timed_out;
      },
      [](const auto &) {}},
    command);
}

void RedundancySwitcherAdapter::on_switcher_status(const ElectionStatus & status)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!uds_connection_established_) {
      RCLCPP_INFO(node_->get_logger(), "UDS path established");
      uds_connection_established_ = true;
    }

    last_election_status_ = status;
    stamp_election_status_ = node_->now();
  }

  check_switcher_connection();

  const auto signals = to_switcher_signals(status.node_state);
  const auto node_state_annotation =
    node_state_to_string(status.node_state) + " leader=" + std::to_string(status.leader_id);
  const auto active_control_unit = to_active_control_unit(status.path_info);
  const auto path_info_annotation = path_info_to_string(status.path_info);
  gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
    Annotated<SwitcherSignals>{signals, node_state_annotation}}});
  gateway_->submit(InputEvent{SetActiveControlUnitEvent{
    Annotated<ActiveControlUnit>{active_control_unit, path_info_annotation}}});
}

// ---------------------------------------------------------------------------
// check_election_status_timeout — Detects timeout on each periodic timer tick.
//
// on_switcher_status is called only when data arrives, so a timeout cannot be
// detected if the Switcher stops sending. This timer handles that monitoring instead.
// ---------------------------------------------------------------------------

void RedundancySwitcherAdapter::check_election_status_timeout()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!stamp_election_status_.has_value()) return;

    const auto now = node_->now();
    const double elapsed_ms = (now - *stamp_election_status_).seconds() * 1000.0;
    timed_out = elapsed_ms > election_status_timeout_milli_;
  }

  if (timed_out) {
    gateway_->submit(InputEvent{
      SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{{false, false, true}, "Switcher data timeout"}}});
  }
}

std::string RedundancySwitcherAdapter::node_state_to_string(uint8_t node_state)
{
  switch (node_state) {
    case 0: return "INITIALIZING";
    case 1: return "ELECTABLE";
    case 2: return "WAIT_FOR_AUTOWARE";
    case 3: return "IN_ELECTION";
    case 4: return "ELECTION_COMPLETED";
    case 5: return "ELECTION_UNCLOSED";
    case 6: return "PATH_NOT_FOUND";
    case 7: return "SELF_INTERRUPTION";
    default: return "UNKNOWN(" + std::to_string(node_state) + ")";
  }
}

// ---------------------------------------------------------------------------
// to_switcher_signals — Maps ElectionStatus.node_state to SwitcherSignals.
//
// Each node_state maps to exactly one of the three signals (mutually exclusive):
//   is_stable:           ELECTABLE(1) / ELECTION_COMPLETED(4)
//   is_self_interrupted: SELF_INTERRUPTION(7)
//   is_faulted:          ELECTION_UNCLOSED(5) / PATH_NOT_FOUND(6)
//   all false:           INITIALIZING(0) / WAIT_FOR_AUTOWARE(2) / IN_ELECTION(3) / unknown
// ---------------------------------------------------------------------------

SwitcherSignals RedundancySwitcherAdapter::to_switcher_signals(uint8_t node_state)
{
  switch (node_state) {
    case 1:  // ELECTABLE
    case 4:  // ELECTION_COMPLETED
      return {true, false, false};
    case 7:  // SELF_INTERRUPTION
      return {false, true, false};
    case 5:  // ELECTION_UNCLOSED
    case 6:  // PATH_NOT_FOUND
      return {false, false, true};
    default:  // INITIALIZING / WAIT_FOR_AUTOWARE / IN_ELECTION / unknown
      return {false, false, false};
  }
}

std::string RedundancySwitcherAdapter::path_info_to_string(uint8_t path_info)
{
  // Bit-field: bit0=main_ecu, bit1=sub_ecu, bit2=main_vcu, bit3=sub_vcu connectivity.
  switch (path_info) {
    case 0: return "unknown";
    case 5: return "main_ecu_to_main_vcu";
    case 6: return "sub_ecu_to_main_vcu";
    case 9: return "main_ecu_to_sub_vcu";
    case 10: return "sub_ecu_to_sub_vcu";
    default: return "unknown(" + std::to_string(path_info) + ")";
  }
}

ActiveControlUnit RedundancySwitcherAdapter::to_active_control_unit(uint8_t path_info)
{
  ActiveControlUnit acu;
  for (int i = 0; i < 4; ++i) {  // check bits for all four units
    if (path_info & (1 << i)) {
      acu.unit_ids.push_back(i);
    }
  }
  return acu;
}

void RedundancySwitcherAdapter::check_switcher_connection()
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (!s_opt.has_value()) {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    node_fault_points_.clear();
    link_fault_points_.clear();
    return;
  }

  const auto & s = *s_opt;
  if (s.node_state == 0 /* INITIALIZING */ || s.node_state == 3 /* IN_ELECTION */) {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    node_fault_points_.clear();
    link_fault_points_.clear();
    return;
  }

  bool autoware_ready = false;
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    autoware_ready = autoware_ready_.has_value() && *autoware_ready_ == AutowareReady::True;
  }
  if (!autoware_ready) {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    node_fault_points_.clear();
    link_fault_points_.clear();
    return;
  }

  std::unordered_set<std::string> node_faults;
  std::unordered_set<std::string> link_faults;

  // Initialize all connections in 'judge' as trusted, then invalidate reports from nodes
  // unreachable from this ECU.
  ElectionStatus judge = s;
  judge.main_ecu_to_main_ecu_connected = judge.main_ecu_to_sub_ecu_connected =
  judge.main_ecu_to_main_vcu_connected = judge.main_ecu_to_sub_vcu_connected =
  judge.sub_ecu_to_main_ecu_connected  = judge.sub_ecu_to_sub_ecu_connected  =
  judge.sub_ecu_to_main_vcu_connected  = judge.sub_ecu_to_sub_vcu_connected  =
  judge.main_vcu_to_main_ecu_connected = judge.main_vcu_to_sub_ecu_connected =
  judge.main_vcu_to_main_vcu_connected = judge.main_vcu_to_sub_vcu_connected =
  judge.sub_vcu_to_main_ecu_connected  = judge.sub_vcu_to_sub_ecu_connected  =
  judge.sub_vcu_to_main_vcu_connected  = judge.sub_vcu_to_sub_vcu_connected  = true;

  // Trust propagation: if the connection from this ECU to a remote node is severed,
  // distrust all reports originating from that node.
  auto distrust = [](bool connected, bool & a, bool & b, bool & c, bool & d) {
    if (!connected) a = b = c = d = false;
  };

  if (is_main_ecu_) {
    if (!s.main_ecu_to_sub_ecu_connected && !s.main_ecu_to_main_vcu_connected &&
        !s.main_ecu_to_sub_vcu_connected) {
      node_faults.insert("main_ecu");
      std::lock_guard<std::mutex> lock(fault_mutex_);
      node_fault_points_ = std::move(node_faults);
      link_fault_points_.clear();
      return;
    }
    distrust(s.main_ecu_to_sub_ecu_connected,
      judge.sub_ecu_to_main_ecu_connected, judge.sub_ecu_to_sub_ecu_connected,
      judge.sub_ecu_to_main_vcu_connected, judge.sub_ecu_to_sub_vcu_connected);
    distrust(s.main_ecu_to_main_vcu_connected,
      judge.main_vcu_to_main_ecu_connected, judge.main_vcu_to_sub_ecu_connected,
      judge.main_vcu_to_main_vcu_connected, judge.main_vcu_to_sub_vcu_connected);
    distrust(s.main_ecu_to_sub_vcu_connected,
      judge.sub_vcu_to_main_ecu_connected, judge.sub_vcu_to_sub_ecu_connected,
      judge.sub_vcu_to_main_vcu_connected, judge.sub_vcu_to_sub_vcu_connected);
  } else {
    if (!s.sub_ecu_to_main_ecu_connected && !s.sub_ecu_to_main_vcu_connected &&
        !s.sub_ecu_to_sub_vcu_connected) {
      node_faults.insert("sub_ecu");
      std::lock_guard<std::mutex> lock(fault_mutex_);
      node_fault_points_ = std::move(node_faults);
      link_fault_points_.clear();
      return;
    }
    distrust(s.sub_ecu_to_main_ecu_connected,
      judge.main_ecu_to_main_ecu_connected, judge.main_ecu_to_sub_ecu_connected,
      judge.main_ecu_to_main_vcu_connected, judge.main_ecu_to_sub_vcu_connected);
    distrust(s.sub_ecu_to_main_vcu_connected,
      judge.main_vcu_to_main_ecu_connected, judge.main_vcu_to_sub_ecu_connected,
      judge.main_vcu_to_main_vcu_connected, judge.main_vcu_to_sub_vcu_connected);
    distrust(s.sub_ecu_to_sub_vcu_connected,
      judge.sub_vcu_to_main_ecu_connected, judge.sub_vcu_to_sub_ecu_connected,
      judge.sub_vcu_to_main_vcu_connected, judge.sub_vcu_to_sub_vcu_connected);
  }

  // Node fault detection: if all incoming directions are either untrusted or actually disconnected → fault
  auto unreachable = [](bool jf, bool af) { return !jf || !af; };

  if (unreachable(judge.sub_ecu_to_main_ecu_connected,  s.sub_ecu_to_main_ecu_connected) &&
      unreachable(judge.main_vcu_to_main_ecu_connected, s.main_vcu_to_main_ecu_connected) &&
      unreachable(judge.sub_vcu_to_main_ecu_connected,  s.sub_vcu_to_main_ecu_connected))
    node_faults.insert("main_ecu");

  if (unreachable(judge.main_ecu_to_sub_ecu_connected, s.main_ecu_to_sub_ecu_connected) &&
      unreachable(judge.main_vcu_to_sub_ecu_connected, s.main_vcu_to_sub_ecu_connected) &&
      unreachable(judge.sub_vcu_to_sub_ecu_connected,  s.sub_vcu_to_sub_ecu_connected))
    node_faults.insert("sub_ecu");

  if (unreachable(judge.main_ecu_to_main_vcu_connected, s.main_ecu_to_main_vcu_connected) &&
      unreachable(judge.sub_ecu_to_main_vcu_connected,  s.sub_ecu_to_main_vcu_connected) &&
      unreachable(judge.sub_vcu_to_main_vcu_connected,  s.sub_vcu_to_main_vcu_connected))
    node_faults.insert("main_vcu");

  if (unreachable(judge.main_ecu_to_sub_vcu_connected, s.main_ecu_to_sub_vcu_connected) &&
      unreachable(judge.sub_ecu_to_sub_vcu_connected,  s.sub_ecu_to_sub_vcu_connected) &&
      unreachable(judge.main_vcu_to_sub_vcu_connected, s.main_vcu_to_sub_vcu_connected))
    node_faults.insert("sub_vcu");

  // Link fault detection: judge=true with both endpoints healthy but actual=false → link fault
  const bool mef = node_faults.count("main_ecu");
  const bool sef = node_faults.count("sub_ecu");
  const bool mvf = node_faults.count("main_vcu");
  const bool svf = node_faults.count("sub_vcu");

  auto mark_link = [&](bool jf, bool af, bool src_f, bool dst_f, const std::string & key) {
    if (jf && !src_f && !dst_f && !af) link_faults.insert(key);
  };

  // clang-format off
  mark_link(judge.main_ecu_to_sub_ecu_connected,  s.main_ecu_to_sub_ecu_connected,  mef, sef, "main_ecu_to_sub_ecu_link");
  mark_link(judge.sub_ecu_to_main_ecu_connected,  s.sub_ecu_to_main_ecu_connected,  sef, mef, "main_ecu_to_sub_ecu_link");
  mark_link(judge.main_ecu_to_main_vcu_connected, s.main_ecu_to_main_vcu_connected, mef, mvf, "main_ecu_to_main_vcu_link");
  mark_link(judge.main_vcu_to_main_ecu_connected, s.main_vcu_to_main_ecu_connected, mvf, mef, "main_ecu_to_main_vcu_link");
  mark_link(judge.main_ecu_to_sub_vcu_connected,  s.main_ecu_to_sub_vcu_connected,  mef, svf, "main_ecu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_main_ecu_connected,  s.sub_vcu_to_main_ecu_connected,  svf, mef, "main_ecu_to_sub_vcu_link");
  mark_link(judge.sub_ecu_to_main_vcu_connected,  s.sub_ecu_to_main_vcu_connected,  sef, mvf, "sub_ecu_to_main_vcu_link");
  mark_link(judge.main_vcu_to_sub_ecu_connected,  s.main_vcu_to_sub_ecu_connected,  mvf, sef, "sub_ecu_to_main_vcu_link");
  mark_link(judge.sub_ecu_to_sub_vcu_connected,   s.sub_ecu_to_sub_vcu_connected,   sef, svf, "sub_ecu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_sub_ecu_connected,   s.sub_vcu_to_sub_ecu_connected,   svf, sef, "sub_ecu_to_sub_vcu_link");
  mark_link(judge.main_vcu_to_sub_vcu_connected,  s.main_vcu_to_sub_vcu_connected,  mvf, svf, "main_vcu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_main_vcu_connected,  s.sub_vcu_to_main_vcu_connected,  svf, mvf, "main_vcu_to_sub_vcu_link");
  // clang-format on

  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    node_fault_points_ = std::move(node_faults);
    link_fault_points_ = std::move(link_faults);
  }
}

bool RedundancySwitcherAdapter::no_data(
  const std::optional<ElectionStatus> & status,
  diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  if (status.has_value()) return false;
  stat.summary(DiagStatus::STALE, "No data received from switcher");
  return true;
}

bool RedundancySwitcherAdapter::is_transitional_state(uint8_t node_state)
{
  return node_state == 0 /* INITIALIZING */ || node_state == 3 /* IN_ELECTION */;
}

void RedundancySwitcherAdapter::update_main_ecu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  bool another_timeout;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = node_fault_points_.count("main_ecu");
  }
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    another_timeout = another_ecu_availability_timeout_;
  }
  if (s.node_state == 7 /* SELF_INTERRUPTION */) {
    stat.summary(DiagStatus::OK, "Main ECU is healthy");
    return;
  }
  // main ecu
  if (is_main_ecu_ && fault) {
    stat.summary(DiagStatus::ERROR, "Main ECU fault detected");
    return;
  }
  // sub ecu
  if (!is_main_ecu_ && another_timeout && fault) {
    stat.summary(DiagStatus::ERROR, "Main ECU fault detected");
    return;
  }
  stat.summary(DiagStatus::OK, "Main ECU is healthy");
}

void RedundancySwitcherAdapter::update_sub_ecu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;
  if (is_transitional_state(s_opt->node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }

  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = node_fault_points_.count("sub_ecu");
  }
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU fault detected" : "Sub ECU is healthy");
}

void RedundancySwitcherAdapter::update_main_vcu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;
  if (is_transitional_state(s_opt->node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }

  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = node_fault_points_.count("main_vcu");
  }
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main VCU fault detected" : "Main VCU is healthy");
}

void RedundancySwitcherAdapter::update_sub_vcu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;
  if (is_transitional_state(s_opt->node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }

  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = node_fault_points_.count("sub_vcu");
  }
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub VCU fault detected" : "Sub VCU is healthy");
}

void RedundancySwitcherAdapter::update_main_ecu_to_sub_ecu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("main_ecu_to_sub_ecu_link");
  }
  stat.add("main_ecu_to_sub_ecu", s.main_ecu_to_sub_ecu_connected);
  stat.add("sub_ecu_to_main_ecu", s.sub_ecu_to_main_ecu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main-Sub ECU link fault detected" : "Main-Sub ECU link is healthy");
}

void RedundancySwitcherAdapter::update_main_ecu_to_main_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("main_ecu_to_main_vcu_link");
  }
  stat.add("main_ecu_to_main_vcu", s.main_ecu_to_main_vcu_connected);
  stat.add("main_vcu_to_main_ecu", s.main_vcu_to_main_ecu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main ECU to Main VCU link fault detected" : "Main ECU to Main VCU link is healthy");
}

void RedundancySwitcherAdapter::update_main_ecu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("main_ecu_to_sub_vcu_link");
  }
  stat.add("main_ecu_to_sub_vcu", s.main_ecu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_main_ecu", s.sub_vcu_to_main_ecu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main ECU to Sub VCU link fault detected" : "Main ECU to Sub VCU link is healthy");
}

void RedundancySwitcherAdapter::update_sub_ecu_to_main_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("sub_ecu_to_main_vcu_link");
  }
  stat.add("sub_ecu_to_main_vcu", s.sub_ecu_to_main_vcu_connected);
  stat.add("main_vcu_to_sub_ecu", s.main_vcu_to_sub_ecu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU to Main VCU link fault detected" : "Sub ECU to Main VCU link is healthy");
}

void RedundancySwitcherAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("sub_ecu_to_sub_vcu_link");
  }
  stat.add("sub_ecu_to_sub_vcu", s.sub_ecu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_sub_ecu", s.sub_vcu_to_sub_ecu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU to Sub VCU link fault detected" : "Sub ECU to Sub VCU link is healthy");
}

void RedundancySwitcherAdapter::update_main_vcu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<ElectionStatus> s_opt;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    s_opt = last_election_status_;
  }
  if (no_data(s_opt, stat)) return;

  const auto & s = *s_opt;
  if (is_transitional_state(s.node_state)) { stat.summary(DiagStatus::OK, "Check skipped: INITIALIZING or IN_ELECTION"); return; }
  bool fault;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault = link_fault_points_.count("main_vcu_to_sub_vcu_link");
  }
  stat.add("main_vcu_to_sub_vcu", s.main_vcu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_main_vcu", s.sub_vcu_to_main_vcu_connected);
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main-Sub VCU link fault detected" : "Main-Sub VCU link is healthy");
}

}  // namespace autoware::redundancy_switcher

PLUGINLIB_EXPORT_CLASS(autoware::redundancy_switcher::RedundancySwitcherAdapter, autoware::redundancy_switcher::IAdapterPlugin)
