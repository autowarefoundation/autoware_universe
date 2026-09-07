# Terminology and Data Flow Definition

This document defines the key terms and data types used in the Redundancy Switcher Interface.

## 1. Core Concepts

### 1.1 InputEvent

**Definition**: A value type representing "what occurred." The Processor's sole input.

**Type**: `std::variant` of the following event structs:

| Event                                   | Source                             | Meaning                                           |
| --------------------------------------- | ---------------------------------- | ------------------------------------------------- |
| `SelfInterruptionEvent`                 | SubSystemAdapter / SwitcherAdapter | This ECU detected an error and must step down.    |
| `ResetEvent`                            | SubSystemAdapter                   | An operator requested a reset.                    |
| `SetAutowareReadyEvent`                 | SubSystemAdapter                   | Autoware readiness changed.                       |
| `SetVelocityStatusEvent`                | SubSystemAdapter                   | Vehicle velocity status changed.                  |
| `SetControlModeEvent`                   | SubSystemAdapter                   | Vehicle control mode changed.                     |
| `SetSwitcherSignalsEvent`               | SwitcherAdapter                    | Switching state updated from the SwitcherAdapter. |
| `SetActiveControlUnitEvent`             | SwitcherAdapter                    | Active control unit updated.                      |
| `SetAnotherEcuAvailabilityTimeoutEvent` | SubSystemAdapter                   | Peer ECU availability timed out or recovered.     |

Adapters create InputEvents and submit them via `EventGateway::submit()` or `submit_request()`.
The Processor has no knowledge of the event source (ROS, UDS, etc.).

---

### 1.2 OutputCommand

**Definition**: A value type representing "what should be done." The Processor's sole output.

**Type**: `std::variant` of the following command structs:

| Command                                      | Consumer         | Meaning                                             |
| -------------------------------------------- | ---------------- | --------------------------------------------------- |
| `LogCommand`                                 | LogAdapter       | Emit a log message at the specified level.          |
| `ResetCommand`                               | SwitcherAdapter  | Send a reset request to the Switcher.               |
| `SelfInterruptionCommand`                    | SwitcherAdapter  | Send a self-interruption request to the Switcher.   |
| `UpdateStatusDiagCommand`                    | DiagAdapter      | Trigger a diagnostic status update.                 |
| `UpdateActiveControlUnitCommand`             | SubSystemAdapter | Publish the active control unit.                    |
| `UpdateAutowareReadyCommand`                 | SwitcherAdapter  | Update the local autoware_ready cache.              |
| `ResetResultCommand`                         | SubSystemAdapter | Return the accept/reject result of a reset request. |
| `UpdateAnotherEcuAvailabilityTimeoutCommand` | SwitcherAdapter  | Update the peer ECU availability timeout cache.     |

The Processor returns a list of OutputCommands from `handle()`.
The CommandBus broadcasts each command to all registered adapters.
Each adapter processes only the command types it owns and ignores the rest.

---

### 1.3 DomainSnapshot

**Definition**: A read-only view of the Processor's current internal state.

```cpp
struct DomainSnapshot {
  std::optional<Annotated<AutowareReady>>    autoware_ready;
  std::optional<Annotated<VelocityStatus>>   velocity_status;
  std::optional<Annotated<ControlMode>>      control_mode;
  std::optional<Annotated<SwitcherSignals>>  switcher;
  std::optional<bool>                        another_ecu_availability_timeout;
};
```

`nullopt` for any field means that data has never been received (startup not yet complete).
Adapters access the snapshot via `EventGateway::snapshot()` for diagnostics and logging.

---

### 1.4 SwitcherSignals

**Definition**: The switching state reported by the SwitcherAdapter, expressed as three mutually exclusive orthogonal signals.

```cpp
struct SwitcherSignals {
  bool is_stable;           // Switcher is ready; self-interruption is possible.
  bool is_self_interrupted; // Self-interruption acknowledged; only reset is accepted.
  bool is_faulted;          // Unrecoverable fault reported; all operations rejected.
};
```

The signals are mutually exclusive. All-false indicates a transitional state
(startup or state change in progress). The concrete state names and mapping logic
reside in the SwitcherAdapter plugin implementation.

---

## 2. Data Flow

```text
┌──────────────────────────────────────────────────────────┐
│  SwitcherAdapter (plugin)                                │
│    UDS recv → SetSwitcherSignalsEvent  ─────────────┐   │
│    UDS recv → SetActiveControlUnitEvent ────────────┤   │
│    ResetCommand        → UDS send                   │   │
│    SelfInterruptionCommand → UDS send               │   │
└─────────────────────────────────────────────────────┼───┘
                                                      │
                                               EventGateway
                                              (mutex-protected)
                                                      │
                                               ┌──────▼──────┐
                                               │  Processor  │
                                               │  (pure C++) │
                                               └──────┬──────┘
                                                      │ OutputCommands
                                               CommandBus (broadcast)
                          ┌───────────────────────────┼───────────────────────┐
                          ▼                           ▼                       ▼
                    LogAdapter               DiagAdapter              SubSystemAdapter
                    (logging)                (diagnostics)            (ROS I/O)
                                                                         │
                                               ┌─────────────────────────┤
                                               │  SetVelocityStatusEvent │
                                               │  SetControlModeEvent    │
                                               │  SetAutowareReadyEvent  │
                                               │  SelfInterruptionEvent  │
                                               │  ResetEvent             │
                                               └─── from Autoware (ROS) ─┘
```

---

## 3. Adapter Responsibilities

| Adapter                      | Inbound (creates InputEvents)                                                                         | Outbound (handles OutputCommands)                |
| ---------------------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| **SubSystemAdapter**         | velocity, control mode, operation mode, availability timeout, reset service, set_initializing service | UpdateActiveControlUnitCommand → publish         |
| **SwitcherAdapter** (plugin) | switching state from the Switcher                                                                     | ResetCommand, SelfInterruptionCommand → Switcher |
| **LogAdapter**               | —                                                                                                     | LogCommand → RCLCPP\_\*                          |
| **DiagAdapter**              | —                                                                                                     | UpdateStatusDiagCommand → diagnostic_updater     |

---

## 4. Key Design Rules

- The Processor has no knowledge of ROS, UDS, or any adapter.
- All state updates go through `EventGateway::submit()` or `submit_request()`.
- `gateway_->snapshot()` is read-only; never submit events from inside `execute()`.
- Each adapter ignores OutputCommand types it does not own.
- Thread safety of `handle()` is the adapter's responsibility (enforced by EventGateway mutex).
