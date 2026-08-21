# SwitcherAdapter Plugin Development Guide

This guide explains how to implement a custom `IAdapterPlugin` for
`autoware_redundancy_switcher_interface`.

A SwitcherAdapter plugin is responsible for translating between your Switcher's
specific protocol (UDS, shared memory, ROS topics, etc.) and the
`SwitcherSignals` / `ActiveControlUnit` abstraction used by the core framework.

**Reference implementations:**

| Implementation                       | Transport  | Use case                        |
| ------------------------------------ | ---------- | ------------------------------- |
| `SimpleSwitcherAdapter`              | ROS topics | Desktop mock for testing        |
| UDS-based adapter (separate package) | UDS (CBOR) | Hardware Switcher in production |

---

## 1. What the Plugin Must Do

### Mandatory

| Responsibility                                    | When                                                |
| ------------------------------------------------- | --------------------------------------------------- |
| Submit `SetSwitcherSignalsEvent`                  | On every state change from the Switcher             |
| Submit `SetSwitcherSignalsEvent{is_faulted=true}` | When the Switcher stops responding (timeout)        |
| Handle `ResetCommand`                             | Forward a reset request to the Switcher             |
| Handle `SelfInterruptionCommand`                  | Forward a self-interruption request to the Switcher |
| Ignore all other `OutputCommand` types            | Always                                              |

### Recommended

| Responsibility                                     | When                                                                     |
| -------------------------------------------------- | ------------------------------------------------------------------------ |
| Submit `SetActiveControlUnitEvent`                 | When the active control unit changes                                     |
| Cache `UpdateAutowareReadyCommand`                 | If the plugin needs to gate Switcher communication on Autoware readiness |
| Cache `UpdateAnotherEcuAvailabilityTimeoutCommand` | If the plugin uses peer ECU availability in its own logic                |
| Publish hardware-specific diagnostics              | If the Switcher reports node/link health information                     |

### The most important responsibility: SwitcherSignals contract

Your plugin must translate Switcher-side state into `SwitcherSignals`:

```cpp
struct SwitcherSignals {
  bool is_stable;           // Switcher is ready; self-interruption is possible
  bool is_self_interrupted; // Self-interruption acknowledged; only reset accepted
  bool is_faulted;          // Unrecoverable fault; all operations rejected
  // all false = transitional (startup or state change in progress)
};
```

These three signals are **mutually exclusive**. The Processor makes all decisions
based solely on these signals; it has no knowledge of your protocol.

---

## 2. IAdapterPlugin Interface

```cpp
// include/redundancy_switcher_interface/plugin/i_adapter_plugin.hpp

class IAdapterPlugin {
public:
  virtual ~IAdapterPlugin() = default;

  // Called once at node startup. Set up your transport (subscribers, threads, timers).
  virtual void initialize(rclcpp::Node* node, std::shared_ptr<EventGateway> gateway) = 0;

  // Called by CommandBus for every OutputCommand. Handle what you own; ignore the rest.
  virtual void execute(const OutputCommand& command) = 0;
};
```

---

## 3. Minimal Implementation

```cpp
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>
#include <redundancy_switcher_interface/ir/input_events.hpp>
#include <redundancy_switcher_interface/ir/output_commands.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace my_package
{

class MySwitcherAdapter : public autoware::redundancy_switcher::IAdapterPlugin
{
public:
  void initialize(rclcpp::Node * node,
                  std::shared_ptr<autoware::redundancy_switcher::EventGateway> gateway) override
  {
    using namespace autoware::redundancy_switcher;

    node_    = node;
    gateway_ = gateway;

    // Read is_main_ecu (already declared by the interface node)
    is_main_ecu_ = node_->has_parameter("is_main_ecu")
      ? node_->get_parameter("is_main_ecu").as_bool()
      : node_->declare_parameter<bool>("is_main_ecu", true);

    // Set up your Switcher transport here
    // (subscribe to topics, open UDS socket, start receive thread, etc.)

    // Set up a timer to detect Switcher timeout
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { check_timeout(); });
  }

  void execute(const autoware::redundancy_switcher::OutputCommand & command) override
  {
    using namespace autoware::redundancy_switcher;
    std::visit(
      overloaded{
        [this](const ResetCommand &) {
          send_reset_to_switcher();
        },
        [this](const SelfInterruptionCommand &) {
          send_self_interruption_to_switcher(is_main_ecu_);
        },
        [](const auto &) { /* ignore */ }},
      command);
  }

private:
  // Called when a status update arrives from the Switcher
  void on_switcher_state(bool is_stable, bool is_self_interrupted, bool is_faulted,
                         const std::string & annotation)
  {
    using namespace autoware::redundancy_switcher;
    gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
      Annotated<SwitcherSignals>{{is_stable, is_self_interrupted, is_faulted}, annotation}}});

    last_received_ = node_->now();
  }

  // Called on each timer tick
  void check_timeout()
  {
    using namespace autoware::redundancy_switcher;
    if (!last_received_.has_value()) return;

    const double elapsed_ms = (node_->now() - *last_received_).seconds() * 1000.0;
    if (elapsed_ms > timeout_milli_) {
      gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
        Annotated<SwitcherSignals>{{false, false, true}, "Switcher timeout"}}});
    }
  }

  void send_reset_to_switcher() { /* your transport */ }
  void send_self_interruption_to_switcher(bool is_main) { /* your transport */ }

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<autoware::redundancy_switcher::EventGateway> gateway_;
  bool is_main_ecu_{true};
  double timeout_milli_{1000.0};
  std::optional<rclcpp::Time> last_received_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_package

PLUGINLIB_EXPORT_CLASS(
  my_package::MySwitcherAdapter,
  autoware::redundancy_switcher::IAdapterPlugin)
```

---

## 4. Event Submission Reference

### SetSwitcherSignalsEvent — call on every Switcher state change

```cpp
gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
  Annotated<SwitcherSignals>{
    {is_stable, is_self_interrupted, is_faulted},
    "human-readable annotation"   // shown in logs and diagnostics
  }
}});
```

**When to submit:**

- On every status message from the Switcher
- When timeout is detected: `{false, false, true}` with annotation `"Switcher timeout"`

**Annotation guidelines:**

- Include the raw state name from your protocol (e.g., `"ELECTABLE leader=0"`)
- Keep it under ~60 characters
- Never leave it empty for is_faulted — the reason helps diagnosis

### SetActiveControlUnitEvent — call when the active ECU changes

```cpp
gateway_->submit(InputEvent{SetActiveControlUnitEvent{
  Annotated<ActiveControlUnit>{
    ActiveControlUnit{{0}},   // unit_ids: e.g. [0]=Main ECU, [1]=Sub ECU
    "path annotation"
  }
}});
```

> The Processor will force `active_control_unit` to empty when `is_self_interrupted`
> or `is_faulted` is true, regardless of what you submit here.
> Submit the actual value from the Switcher; the Processor handles the override.

### Caching UpdateAutowareReadyCommand (optional)

If your Switcher needs to know whether Autoware is ready before accepting commands:

```cpp
// In execute():
[this](const UpdateAutowareReadyCommand & cmd) {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  autoware_ready_ = cmd.value;
},
```

Use the cached value when deciding whether to send certain requests to the Switcher.

### Caching UpdateAnotherEcuAvailabilityTimeoutCommand (optional)

If your Switcher needs to react when the peer ECU's availability times out:

```cpp
[this](const UpdateAnotherEcuAvailabilityTimeoutCommand & cmd) {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  another_ecu_availability_timeout_ = cmd.timed_out;
},
```

---

## 5. CommandBus — Which Commands Reach Your Plugin

All `OutputCommand` types are broadcast to every registered adapter.
Your plugin will receive commands that are not meant for it; **you must ignore them**.

```cpp
void execute(const OutputCommand & command) override
{
  std::visit(
    overloaded{
      [this](const ResetCommand &)            { /* handle */ },
      [this](const SelfInterruptionCommand &) { /* handle */ },
      // Cache if needed:
      [this](const UpdateAutowareReadyCommand & cmd)               { /* optional */ },
      [this](const UpdateAnotherEcuAvailabilityTimeoutCommand & cmd) { /* optional */ },
      // All others: ignore
      [](const auto &) {}},
    command);
}
```

**Commands you typically care about:**

| Command                                      | Action                                                                   |
| -------------------------------------------- | ------------------------------------------------------------------------ |
| `ResetCommand`                               | Send reset to the Switcher                                               |
| `SelfInterruptionCommand`                    | Send self-interruption to the Switcher (use `is_main_ecu` to select ECU) |
| `UpdateAutowareReadyCommand`                 | Cache if needed for Switcher-side gating                                 |
| `UpdateAnotherEcuAvailabilityTimeoutCommand` | Cache if needed                                                          |

**Commands you should ignore** (handled by other adapters):

| Command                          | Handled by       |
| -------------------------------- | ---------------- |
| `LogCommand`                     | LogAdapter       |
| `UpdateStatusDiagCommand`        | DiagAdapter      |
| `UpdateActiveControlUnitCommand` | SubSystemAdapter |
| `ResetResultCommand`             | SubSystemAdapter |

---

## 6. Thread Safety

> **Rule**: Never call `gateway_->submit()` while holding any adapter mutex.
> Never call `gateway_->submit()` synchronously from within `execute()`.

Typical pattern for concurrent receive threads:

```cpp
// In receive callback (e.g., UDS receive thread, ROS subscription callback):
void on_switcher_message(...)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    // Update cached state only
    last_status_ = ...;
    last_received_ = node_->now();
  }
  // gateway->submit() OUTSIDE the lock
  gateway_->submit(InputEvent{SetSwitcherSignalsEvent{...}});
}

// In execute() (called by CommandBus, may run on a different thread):
void execute(const OutputCommand & command)
{
  // Cache policy updates under policy_mutex_
  // Do NOT call gateway_->submit() here
  // Publishing (pub->publish()) is OK here
}
```

See [ADAPTER_THREAD_SAFETY.md](ADAPTER_THREAD_SAFETY.md) for the full guide,
recommended mutex split, and deadlock checklist.

---

## 7. pluginlib Registration

### plugins.xml

```xml
<library path="my_switcher_package">
  <class
    name="my_package::MySwitcherAdapter"
    type="my_package::MySwitcherAdapter"
    base_class_type="autoware::redundancy_switcher::IAdapterPlugin">
    <description>My custom Switcher adapter.</description>
  </class>
</library>
```

### CMakeLists.txt

```cmake
pluginlib_export_plugin_description_file(
  autoware_redundancy_switcher_interface plugins.xml)

ament_auto_add_library(my_switcher_package SHARED
  src/my_switcher_adapter.cpp)
```

### package.xml

```xml
<depend>autoware_redundancy_switcher_interface</depend>
<depend>pluginlib</depend>
```

### Runtime configuration

```yaml
# my_package/config/main.param.yaml
/**:
  ros__parameters:
    switcher_plugin: "my_package::MySwitcherAdapter"
    # Your plugin-specific parameters here
```

---

## 8. Comparison: Minimal vs Full Implementation

The table below contrasts the desktop mock with a production-grade hardware plugin to show
which features are optional vs necessary for real deployment.

| Aspect                                              | SimpleSwitcherAdapter (mock)       | Production hardware plugin                                          |
| --------------------------------------------------- | ---------------------------------- | ------------------------------------------------------------------- |
| Transport                                           | ROS topics                         | Hardware-specific (e.g. UDS, shared memory)                         |
| Switcher state source                               | Encoded topic message              | Hardware protocol message                                           |
| Timeout detection                                   | Not needed (mock doesn't time out) | Required — timer polls last receive timestamp                       |
| Caches `UpdateAutowareReadyCommand`                 | No                                 | Recommended if plugin gates Switcher behavior on Autoware readiness |
| Caches `UpdateAnotherEcuAvailabilityTimeoutCommand` | No                                 | Recommended if plugin reacts to peer ECU availability               |
| Hardware-specific diagnostics                       | No                                 | Recommended — publish node/link health from hardware                |
| Dedicated receive thread                            | No (ROS executor)                  | Typically yes for blocking transports                               |
| Mutex count                                         | 1 (annotation cache)               | 3 (status / policy / diagnostics)                                   |

---

## 9. Checklist Before Shipping

- [ ] `initialize()` validates `node` and `gateway` are not null
- [ ] `execute()` handles `ResetCommand` and `SelfInterruptionCommand`
- [ ] `execute()` ignores all other command types
- [ ] `SetSwitcherSignalsEvent` is submitted on every Switcher state change
- [ ] Timeout detection is implemented via a timer (not relying on callbacks alone)
- [ ] Timeout submits `{false, false, true}` (is_faulted)
- [ ] `gateway_->submit()` is never called while holding a mutex
- [ ] `gateway_->submit()` is never called from within `execute()`
- [ ] All publish calls are outside mutex scope
- [ ] `plugins.xml` uses `autoware::redundancy_switcher::IAdapterPlugin` as base class type
- [ ] `switcher_plugin` param value matches the class name in `plugins.xml`
