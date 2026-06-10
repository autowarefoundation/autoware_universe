# autoware_mrm_reset_manager

## Purpose

`autoware_mrm_reset_manager` manages MRM reset orchestration by:

- forwarding `reset_mrm` requests to system reset services, and
- coordinating initialization/reset timing from AD API states.

This package is focused on a single ECU path (no main/sub split logic).

---

## Inner-workings / Algorithms

### 1. `reset_mrm` forwarding

When `~/input/reset_mrm` is called, the node:

1. Calls `~/output/reset_diag_graph`
2. Calls `~/output/reset_redundancy_switcher` (only when `is_redundant=true`)
3. Returns success when all enabled calls succeed

### 2. Initialization state machine

At startup, a 1-second timer advances the following state machine:

| State | Action |
|---|---|
| `WAIT_SERVICES_READY` | Wait until all required output services are available |
| `SET_AGGREGATOR_INIT` | Call `set_aggregator_initializing(true)` |
| `RESET_SWITCHER` | Call `reset_redundancy_switcher()` |
| `SET_SWITCHER_IFACE_INIT` | Call `set_redundancy_switcher_interface_initializing(true)` |
| `DONE` | Stop init timer and start periodic 5-second check |

### 3. Ready-state transition

The node subscribes to:

- localization initialization state,
- route state,
- operation mode state.

When all of the following are true:

- localization is initialized,
- route is set,
- Autoware control is enabled,

it performs the following actions only when `enable_autoware_ready_actions=true`:

1. `set_aggregator_initializing(false)`
2. `reset_redundancy_switcher()`
3. `set_redundancy_switcher_interface_initializing(false)`

If the system is still initializing and not yet ready, a 5-second periodic timer retries `reset_redundancy_switcher()`.

---

## Inputs / Outputs

### Input Service

| Topic | Type | Description |
|---|---|---|
| `~/input/reset_mrm` | `tier4_external_api_msgs::srv::ResetMrm` | External MRM reset request |

### Input Subscriptions

| Topic | Type | Description |
|---|---|---|
| `~/input/localization_initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` | Localization readiness |
| `~/input/route_state` | `autoware_adapi_v1_msgs::msg::RouteState` | Route readiness |
| `~/input/operation_mode_state` | `autoware_adapi_v1_msgs::msg::OperationModeState` | Autoware control state |

### Output Service Clients

| Topic | Type | Description |
|---|---|---|
| `~/output/reset_diag_graph` | `tier4_system_msgs::srv::ResetDiagGraph` | Resets diagnostics graph |
| `~/output/reset_redundancy_switcher` | `tier4_system_msgs::srv::ResetRedundancySwitcher` | Resets redundancy switcher |
| `~/output/set_aggregator_initializing` | `std_srvs::srv::SetBool` | Sets aggregator initializing flag |
| `~/output/set_redundancy_switcher_interface_initializing` | `std_srvs::srv::SetBool` | Sets switcher-interface initializing flag |

---

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `service_timeout_ms` | int | 200 | Timeout for each service call |
| `is_redundant` | bool | true | Enables `reset_redundancy_switcher` calls. When `false`, all reset-redundancy-switcher calls are skipped and treated as success. |
| `enable_autoware_ready_actions` | bool | true | Enables service calls triggered by Autoware-ready conditions (localization initialized, route set, Autoware control enabled). |

---

## Launch

Launch file:

- `launch/mrm_reset_manager.launch.xml`

Main launch arguments:

| Argument | Default |
|---|---|
| `mrm_reset_manager_param_file` | `$(find-pkg-share autoware_mrm_reset_manager)/config/mrm_reset_manager.param.yaml` |
| `is_redundant` | `true` |
| `enable_autoware_ready_actions` | `true` |
| `input_reset_mrm` | `/system/mrm/reset` |
| `input_localization_initialization_state` | `/api/localization/initialization_state` |
| `input_route_state` | `/api/routing/state` |
| `input_operation_mode_state` | `/api/operation_mode/state` |
| `output_reset_diag_graph` | `/system/mrm_reset_manager/diagnostics_graph/reset` |
| `output_reset_redundancy_switcher` | `/system/mrm_reset_manager/redundancy_switcher/reset` |
| `output_set_aggregator_initializing` | `/system/mrm_reset_manager/aggregator/set_initializing` |
| `output_set_redundancy_switcher_interface_initializing` | `/system/mrm_reset_manager/redundancy_switcher_interface/set_initializing` |

---

