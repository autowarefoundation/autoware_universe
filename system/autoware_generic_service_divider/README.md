# autoware_generic_service_divider

## Purpose

Autoware can be deployed in a redundant configuration, where the same set of nodes runs more than once, typically in separate ROS 2 domains (for example a `main` ECU and a `sub` ECU) so that one side can take over when the other fails.

Service calls do not fan out on their own. A topic publisher reaches every subscriber, but a service client talks to exactly one server. That is a problem for the control-plane services that must be applied to _both_ sides at the same time, such as changing the operation mode, requesting a control mode, or resetting the diagnostic graph. Without a fan-out mechanism every caller would have to know how many redundant instances exist, hold one client per instance, call them all, and then decide what to answer when only some of them succeed.

This node advertises a single **input service**, forwards each incoming request to **N configured output services**, waits for all of them, and returns one aggregated response to the original caller. Callers stay unaware of the redundancy, and the fan-out policy is concentrated in one place.

```text
                          +---------------------------------------+
                          |        generic_service_divider        |
 caller --request--> /system/operation_mode/change_operation_mode  |
                          |            |                          |
                          |            +--> /main/system/operation_mode/change_operation_mode
                          |            |         (primary, timeout 200 ms)
                          |            +--> /sub/system/operation_mode/change_operation_mode
                          |            |         (timeout 500 ms)
 caller <--response-- aggregated result |                          |
                          +---------------------------------------+
```

## Inner-workings / Algorithms

### Architecture

The node itself contains no service-specific logic. Everything is driven by divider plugins loaded through `pluginlib`.

| Component                          | File                                                | Role                                                                                                                                                            |
| ---------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GenericServiceDividerNode`        | `src/generic_service_divider_node.cpp`              | Loads the plugins listed in the `plugins` parameter, calls `setup_service_division()` on each, and publishes the startup diagnostics.                           |
| `ServiceDividerPluginBase`         | `src/service_divider_plugin_base.cpp`               | Implements the entire fan-out flow: startup gating, request forwarding, per-output timeouts, response aggregation, and cleanup.                                 |
| Divider plugins                    | `plugins/*.cpp`                                     | Declare _what_ to divide: the service type, the input service name, the output service configuration, how to judge success, and how to build an error response. |
| `GenericService` / `GenericClient` | `src/generic_service.cpp`, `src/generic_client.cpp` | Type-erased service server and client, so the base class can forward a request without being compiled against the concrete service type.                        |
| `service_typesupport_helpers`      | `src/service_typesupport_helpers.cpp`               | Resolves the introspection typesupport library from a service type string and allocates zero-initialized messages.                                              |

Because the request payload is handled as `std::shared_ptr<void>`, message allocation and initialization are resolved at runtime from the type string (for example `autoware_system_msgs/srv/ChangeOperationMode`) through `rosidl_typesupport_introspection_cpp`. Adding support for a new service type therefore does not require any change to the fan-out logic.

### Startup gating (fail-closed)

`setup_service_division()` creates one `GenericClient` per output service and then calls `try_start_input_service()`.

**The input service is advertised only after every output service server is available.** While any output server is missing, the node

- does not advertise the input service, so callers see "service not available" instead of a request that silently reaches only a subset of the outputs,
- retries every 500 ms with a wall timer,
- logs `Service divider: waiting for output service servers before advertising '<input>' (ready=k/n, waiting=[...])` at most once per 5 s, and
- reports `ERROR` on the diagnostic status.

Once every output server is ready, the input service is advertised, the retry timer is cancelled, and the node logs `Service divider: <input> -> <n> outputs (type: <service type>)`.

### Request forwarding

For each incoming request the base class

1. creates a _pending division_ and registers it under an incrementing id that appears in every related log line as `Service divider[<id>]`,
2. forwards the **unmodified** request to every output service concurrently, and
3. arms a per-output wall timer with that output's `timeout_ms`.

Each output client is placed in its own `MutuallyExclusive` callback group, and the input service is placed in a `Reentrant` callback group, so concurrent input calls are tracked independently. The node executable runs on a `MultiThreadedExecutor`.

Whichever comes first, the response callback or the timeout timer, marks the output as completed through `mark_output_completed()`; the loser of that race returns without touching the state. When the last output completes, the timers are cancelled and the response is finalized.

### Response aggregation

| Condition                                                          | Response returned to the caller                                                   |
| ------------------------------------------------------------------ | --------------------------------------------------------------------------------- |
| All outputs succeeded                                              | The **primary output's response, verbatim**                                       |
| The primary responded, but at least one output failed or timed out | Error response with the message `One or more output services failed or timed out` |
| The primary did not respond (timeout or send failure)              | Error response with the message `Primary service did not respond`                 |

Success is judged per service type by the plugin through `is_response_success()`, for example `response.status.success` or `response.success`. Error responses are built by the plugin through `create_error_response()`; for service types that carry `autoware_common_msgs/msg/ResponseStatus`, the code is set to `ResponseStatus::SERVICE_TIMEOUT`.

## Inputs / Outputs

The service names below are the defaults in `config/generic_service_divider.param.yaml`. All of them are configurable, and a service pair exists only when its plugin is listed in the `plugins` parameter.

### Input / Output services

| Interface type | Name                                                        | Type                                             | Description                                 |
| -------------- | ----------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------- |
| service        | `/system/operation_mode/change_operation_mode`              | `autoware_system_msgs/srv/ChangeOperationMode`   | Input service advertised to callers         |
| client         | `/{main,sub}/system/operation_mode/change_operation_mode`   | `autoware_system_msgs/srv/ChangeOperationMode`   | Output services the request is forwarded to |
| service        | `/system/operation_mode/change_autoware_control`            | `autoware_system_msgs/srv/ChangeAutowareControl` | Input service                               |
| client         | `/{main,sub}/system/operation_mode/change_autoware_control` | `autoware_system_msgs/srv/ChangeAutowareControl` | Output services                             |
| service        | `/control/control_mode_request`                             | `autoware_vehicle_msgs/srv/ControlModeCommand`   | Input service                               |
| client         | `/{main,sub}/control/control_mode_request`                  | `autoware_vehicle_msgs/srv/ControlModeCommand`   | Output services                             |
| service        | `/diagnostics_graph/reset`                                  | `tier4_system_msgs/srv/ResetDiagGraph`           | Input service                               |
| client         | `/{main,sub}/diagnostics_graph/reset`                       | `tier4_system_msgs/srv/ResetDiagGraph`           | Output services                             |
| service        | `/system/redundancy_switcher/reset`                         | `tier4_system_msgs/srv/ResetRedundancySwitcher`  | Input service                               |
| client         | `/{main,sub}/system/redundancy_switcher/reset`              | `tier4_system_msgs/srv/ResetRedundancySwitcher`  | Output services                             |
| publisher      | `/diagnostics`                                              | `diagnostic_msgs/msg/DiagnosticArray`            | Startup readiness of the divider            |

### Diagnostics

The node publishes the diagnostic status `service_startup_readiness` with the hardware id `generic_service_divider`, and forces an update at 1 Hz.

| Key                                  | Value                                                            |
| ------------------------------------ | ---------------------------------------------------------------- |
| `plugin_count`                       | Number of successfully loaded plugins                            |
| `output_services_ready`              | `<ready>/<total>` counted across all plugins                     |
| `input_service.<input service name>` | `ready` or `waiting`, one entry per plugin                       |
| `waiting_output_services`            | `<input> -> [<output>, <output>] \| <input> -> [...]`, or `none` |

| Level   | Message                                                          | Condition                                 |
| ------- | ---------------------------------------------------------------- | ----------------------------------------- |
| `OK`    | `All service checks completed`                                   | Every input service is advertised         |
| `ERROR` | `Waiting for output services before input service advertisement` | At least one input service is still gated |
| `OK`    | `No plugins configured`                                          | The `plugins` parameter is empty          |

## Parameters

| Parameter name                         | Type       | Default           | Description                                                                             |
| -------------------------------------- | ---------- | ----------------- | --------------------------------------------------------------------------------------- |
| `plugins`                              | `string[]` | `[]`              | Divider plugin class names to load. When empty, the node does nothing and reports `OK`. |
| `<prefix>.input_service`               | `string`   | Plugin specific   | Service name advertised to callers.                                                     |
| `<prefix>.output_services.names`       | `string[]` | `[]`              | Output services each request is forwarded to.                                           |
| `<prefix>.output_services.primaries`   | `bool[]`   | `false` per entry | Which output's response is returned when all outputs succeed.                           |
| `<prefix>.output_services.timeouts_ms` | `int[]`    | `200` per entry   | Per-output timeout in milliseconds.                                                     |

`names`, `primaries`, and `timeouts_ms` are matched **by index**. Entries missing from `primaries` or `timeouts_ms` fall back to `false` and `200` respectively.

`<prefix>` is fixed per plugin class:

| Plugin class                                                                 | Parameter prefix                                 | Service type                                     |
| ---------------------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------ |
| `generic_service_divider::ChangeOperationModeDivider`                        | `change_operation_mode`                          | `autoware_system_msgs/srv/ChangeOperationMode`   |
| `generic_service_divider::ChangeAutowareControlDivider`                      | `change_autoware_control`                        | `autoware_system_msgs/srv/ChangeAutowareControl` |
| `generic_service_divider::ControlModeRequestDivider`                         | `control_mode_request`                           | `autoware_vehicle_msgs/srv/ControlModeCommand`   |
| `generic_service_divider::ResetDiagGraphDivider`                             | `reset_diag_graph`                               | `tier4_system_msgs/srv/ResetDiagGraph`           |
| `generic_service_divider::ResetRedundancySwitcherDivider`                    | `reset_redundancy_switcher`                      | `tier4_system_msgs/srv/ResetRedundancySwitcher`  |
| `generic_service_divider::SetAggregatorInitializingDivider`                  | `set_aggregator_initializing`                    | `std_srvs/srv/SetBool`                           |
| `generic_service_divider::SetRedundancySwitcherInterfaceInitializingDivider` | `set_redundancy_switcher_interface_initializing` | `std_srvs/srv/SetBool`                           |
| `generic_service_divider::EkfTriggerNodeDivider`                             | `ekf_trigger_node`                               | `std_srvs/srv/SetBool`                           |

## Usage

### Launch

```bash
ros2 launch autoware_generic_service_divider generic_service_divider.launch.xml \
  config_file:=/path/to/generic_service_divider.param.yaml
```

`config_file` defaults to the packaged `config/generic_service_divider.param.yaml`.

### Configuration example

```yaml
/**:
  ros__parameters:
    plugins:
      - generic_service_divider::ChangeOperationModeDivider
      - generic_service_divider::ControlModeRequestDivider

    change_operation_mode:
      input_service: /system/operation_mode/change_operation_mode
      output_services:
        names:
          - /main/system/operation_mode/change_operation_mode
          - /sub/system/operation_mode/change_operation_mode
        primaries:
          - true
          - false
        timeouts_ms:
          - 200
          - 500

    control_mode_request:
      input_service: /control/control_mode_request
      output_services:
        names:
          - /main/control/control_mode_request
          - /sub/control/control_mode_request
        primaries:
          - true
          - false
        timeouts_ms:
          - 200
          - 500
```

### Checking the state at runtime

```bash
# Is the input service advertised yet?
ros2 service list | grep change_operation_mode

# Why is it not advertised?
ros2 topic echo /diagnostics --once

# Call the input service and see the aggregated response
ros2 service call /system/operation_mode/change_operation_mode \
  autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}"
```

Every step of a division is logged with the pending id, which makes it possible to follow one request across the outputs:

```text
Service divider[7]: call received on '/system/operation_mode/change_operation_mode'
Service divider[7]: forwarding call to '/main/...' (primary=true, timeout_ms=200)
Service divider[7]: forwarding call to '/sub/...' (primary=false, timeout_ms=500)
Service divider[7]: response from '/main/...'
Service divider[7]: timeout waiting for response from '/sub/...'
Service divider: '/sub/...' timed out
Service divider: at least one output failed/timed out, returning error response (primary='/main/...')
```

## Adding a divider for a new service type

1. Add `plugins/<name>_divider.cpp` deriving from `ServiceDividerPluginBase` and override:

   ```cpp
   void initialize(rclcpp::Node::SharedPtr node) override;   // declare the parameters
   std::string service_type() const override;                // "pkg/srv/Type"
   std::string input_service_name() const override;
   std::vector<OutputServiceConfig> output_services() const override;
   bool is_response_success(const void * response) const override;
   std::shared_ptr<void> create_error_response(const std::string & message) const override;
   // optional, used for detailed logs
   std::string format_request(const void * request) const override;
   std::string format_response(const void * response) const override;
   ```

2. Register the class with `PLUGINLIB_EXPORT_CLASS`.
3. Add the class to `plugins.xml`.
4. Add the source file to the `${PROJECT_NAME}_plugins` library in `CMakeLists.txt`.
5. Add the message package to `package.xml` if it is not already a dependency.

The fan-out logic itself is inherited, so no change to `ServiceDividerPluginBase` is needed. Use an existing plugin such as `plugins/change_operation_mode_divider.cpp` as a template; a plugin is roughly 70 lines.

## Assumptions / Known limits

1. **The input service is unavailable until every output server is up.** This is deliberate, since dividing into a subset of the outputs would apply a request to only part of a redundant system. The consequence is that one missing redundant ECU blocks the input service entirely. Monitor the `service_startup_readiness` diagnostic to distinguish "gated" from "broken".
2. **The input service is never un-advertised.** Once advertised, it stays advertised even if an output server disappears later. Calls then take the timeout path and return an error response.
3. **Only the primary output's response is propagated.** Non-primary responses are logged and then discarded.
4. **Aggregation is all-or-nothing.** If any output fails or times out, the caller receives a generic error response. Which output failed is visible only in the node's log, not in the response.
5. **The error code is coarse.** For service types that carry `ResponseStatus`, error responses always use `SERVICE_TIMEOUT` even when the real cause was a failure response. For service types whose response is only a boolean, such as `std_srvs/srv/SetBool` and `autoware_vehicle_msgs/srv/ControlModeCommand`, the reason is lost entirely and only `success = false` remains.
6. **If several outputs are marked `primary: true`, the last one in the list wins.** The configuration is not validated.
7. **Requests are forwarded unmodified.** The node does not remap, rewrite, or filter payloads, so every output must accept the identical request.
8. **A timed-out request is not cancelled.** The timeout only stops the divider from waiting. A response arriving after the timeout is dropped.
9. **One instance per service type per node.** Parameter prefixes are hardcoded in each plugin class, so the same service type cannot be divided twice within one node, for example two different `ChangeOperationMode` inputs. Run a second node instance instead.
10. **No retry and no QoS tuning.** Each output is called exactly once per input request with the default service QoS. Only the timeout is configurable.
11. **`GenericService` and `GenericClient` are a Humble-specific workaround.** `rclcpp::GenericClient` does not exist in Humble, so a type-erased client and server are bundled here, derived from the upstream `rclcpp` implementation. They depend on `rclcpp` internals, for example the fact that Humble's `ServiceBase` does not allocate `service_handle_` itself. On Jazzy and later they should be replaced by `rclcpp::GenericClient`, as marked by the `TODO(Jazzy)` comment in `include/generic_service_divider/generic_client.hpp`.
12. **Plugin load failures are not fatal.** A plugin that throws during `createSharedInstance()`, `initialize()`, or `setup_service_division()` is logged as `ERROR` and skipped; the node keeps running with the remaining plugins. Check `plugin_count` in the diagnostics against the configured list.
13. **The shipped configuration file has an unused block.** `config/generic_service_divider.param.yaml` declares the EKF trigger settings under `trigger_node:`, while `EkfTriggerNodeDivider` reads the `ekf_trigger_node.*` prefix. `EkfTriggerNodeDivider` is not in the default `plugins` list, so nothing breaks today, but the block must be renamed before enabling that plugin.

## Tests

```bash
colcon test --packages-select autoware_generic_service_divider
colcon test-result --verbose
```

`test/test_generic_service_divider.cpp` loads `ChangeOperationModeDivider` against mock output servers and covers:

| Test                                                   | Covers                                                                                                               |
| ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------- |
| `BothServersSucceed`                                   | Fan-out to two outputs and propagation of the primary response                                                       |
| `SubServerFails`, `MainServerFails`, `BothServersFail` | Per-output failure leading to an error response                                                                      |
| `SubServerTimeout`                                     | An output server disappearing after advertisement, so the timeout path is reached                                    |
| `InputServiceWaitsForOutputServers`                    | Startup gating, the `waiting_output_services` content, and the transition to `ready` when the missing server appears |
| `ConsecutiveCallsSucceed`                              | Cleanup of the pending division between two calls                                                                    |

`SubServerTimeout` starts both output servers first and stops the sub server only after the input service has been advertised. Removing that setup makes the test unreachable, because the input service would never be advertised in the first place.
