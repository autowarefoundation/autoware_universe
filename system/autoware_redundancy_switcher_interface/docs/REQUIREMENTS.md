# Functional Requirements — autoware_redundancy_switcher_interface

## 1. Purpose

This package provides the framework for managing redundancy switching in an Autoware-based vehicle system. It mediates between the Autoware stack (lower system) and a hardware Switcher (upper system) to decide when and how to transfer control authority between redundant ECUs.

---

## 2. Functional Requirements

### FR-01: System state aggregation

The system shall continuously receive and maintain the following state from the Autoware stack:

| Input                 | Source                          | Values                |
| --------------------- | ------------------------------- | --------------------- |
| Autoware readiness    | `/set_initializing` service     | `False` / `True`      |
| Vehicle velocity      | velocity topic                  | `Stopped` / `Moving`  |
| Vehicle control mode  | control mode topic              | `Manual` / `Auto`     |
| Peer ECU availability | command mode availability topic | available / timed-out |

Each state field shall be `nullopt` until the first message is received (startup not yet complete).

---

### FR-02: Switcher state reception

The system shall receive the switching state from a SwitcherAdapter plugin and represent it as three mutually exclusive boolean signals:

| Signal                | Meaning                                                               |
| --------------------- | --------------------------------------------------------------------- |
| `is_stable`           | Switching is complete; self-interruption is possible                  |
| `is_self_interrupted` | A self-interruption has been acknowledged; only reset is accepted     |
| `is_faulted`          | An unrecoverable fault has been reported; all operations are rejected |
| all false             | Transitional state — startup or state change in progress              |

The SwitcherAdapter plugin is responsible for translating hardware-specific state into these three signals.

---

### FR-03: Active control unit management

The system shall maintain and publish the currently active control unit (ECU/VCU IDs).

When the switcher state transitions to `is_self_interrupted` or `is_faulted`, the active control unit shall be forced to empty regardless of what the SwitcherAdapter reports.

---

### FR-04: Self-interruption

The system shall support a self-interruption request from the Autoware stack. Self-interruption requests this ECU to voluntarily step down from active control.

**Acceptance conditions (all must hold):**

1. `autoware_ready = True`
2. `control_mode = Auto`
3. Switcher data has been received (not nullopt)
4. `is_stable = true` (not already interrupted or faulted, not transitional)

**Rejection cases:**

| Condition               | Log level | Reason                         |
| ----------------------- | --------- | ------------------------------ |
| `autoware_ready ≠ True` | Debug     | Autoware is not ready          |
| `control_mode ≠ Auto`   | Debug     | Autoware is not in control     |
| No switcher data        | Warn      | Startup not yet complete       |
| `is_self_interrupted`   | Debug     | Already interrupted            |
| `is_faulted`            | Error     | Switcher fault                 |
| Transitional            | Info      | Switcher in transitional state |

---

### FR-05: Reset

The system shall support a reset request to restore normal operation after self-interruption.

**Processing rules (evaluated in order):**

| Condition                               | Result                  | ResponseStatus               |
| --------------------------------------- | ----------------------- | ---------------------------- |
| `velocity = Moving`                     | Rejected (Ignored)      | NO_EFFECT                    |
| `autoware_ready ≠ True` (incl. nullopt) | **Accepted**            | SUCCESS                      |
| No switcher data                        | Rejected (Error)        | UNKNOWN                      |
| `is_self_interrupted`                   | **Accepted**            | SUCCESS                      |
| `is_stable`                             | Rejected (NotNecessary) | **SUCCESS** (already stable) |
| `is_faulted`                            | Rejected (Error)        | UNKNOWN                      |
| Transitional                            | Rejected (Error)        | UNKNOWN                      |

The result is returned synchronously to the caller (SubSystemAdapter) via `ResetResultCommand`.

---

### FR-06: Peer ECU availability monitoring

The system shall detect when `CommandModeAvailability` messages from the peer ECU have not arrived within a configurable timeout interval.

- Rising edge (false→true): Submit `SetAnotherEcuAvailabilityTimeoutEvent{timed_out=true}`
- Falling edge (true→false): Submit `SetAnotherEcuAvailabilityTimeoutEvent{timed_out=false}`

---

### FR-07: Diagnostics

The system shall publish a `diagnostic_updater` item named `redundancy_switcher_interface_status`
with hardware ID `{main,sub}_ecu_redundancy_switcher_interface`, reflecting the aggregated
state of the four fields below.

| Field (key)        | OK                    | WARN                                                            | ERROR                                         |
| ------------------ | --------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `switcher_signals` | `is_stable`           | `is_self_interrupted`, transitional (before timeout), `nullopt` | `is_faulted`, transitional (timeout exceeded) |
| `autoware_ready`   | `False` or `True`     | `nullopt`                                                       | —                                             |
| `velocity_status`  | `Stopped` or `Moving` | `nullopt`                                                       | —                                             |
| `control_mode`     | `Manual` or `Auto`    | `nullopt`                                                       | —                                             |

The overall summary level is the worst level across all four fields.

A transitional switcher state that persists longer than `diag.transitional_timeout_milli` (ms)
shall be reported as ERROR.

See [DESIGN.md Section 12](DESIGN.md#12-diagadapter--diagnostic-output) for the full level mapping and message format.

---

### FR-08: Logging

All state changes and decision outcomes shall be emitted as `LogCommand` with appropriate log levels:

- `Debug`: Rejections due to expected/normal conditions
- `Info`: State changes, acceptances
- `Warn`: Unexpected but recoverable conditions
- `Error`: Fault states, programming errors

---

### FR-09: Plugin extensibility for Switcher

The Switcher-side adapter shall be loaded as a `pluginlib` plugin at runtime. This decouples the core logic from hardware-specific UDS/topic-based switching protocols.

When `is_redundant = false`, no plugin is loaded and a permanently stable signal is injected instead.

---

### FR-10: Non-redundant mode

When `is_redundant = false` (single-ECU system), the switcher plugin shall not be loaded. The system shall behave as if the switcher is permanently stable:

- Diagnostics: OK
- Reset requests: SUCCESS (not necessary)

---

### FR-11: Thread safety

All concurrent access from multiple adapter threads shall be safe. The EventGateway shall serialize Processor state transitions using a mutex. CommandBus dispatch shall run outside the lock to prevent blocking during I/O operations in adapters.

---

## 3. Configuration Parameters

| Parameter                         | Package            | Default                 | Description                             |
| --------------------------------- | ------------------ | ----------------------- | --------------------------------------- |
| `is_redundant`                    | interface          | `true`                  | Enable redundant mode                   |
| `is_main_ecu`                     | interface / plugin | required                | ECU role (main/sub)                     |
| `diag.transitional_timeout_milli` | interface          | 2000.0                  | Transitional state ERROR threshold (ms) |
| `availability_timeout_milli`      | interface          | 200.0                   | Peer ECU availability timeout (ms)      |
| `switcher_plugin`                 | interface          | (required if redundant) | pluginlib class name                    |

---

## 4. Non-functional Requirements

- **Determinism**: Given the same sequence of InputEvents, the Processor shall always produce the same OutputCommands and state.
- **No exception propagation**: `Processor::handle()` shall not propagate exceptions; errors are expressed as OutputCommands.
- **ROS independence of core logic**: `Processor`, `EventGateway`, and `CommandBus` shall have no ROS dependencies, enabling pure C++ unit tests.
