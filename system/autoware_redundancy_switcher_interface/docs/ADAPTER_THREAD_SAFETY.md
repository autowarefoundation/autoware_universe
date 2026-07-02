# Adapter Thread Safety Guide

This package allows callbacks and `execute()` to run concurrently depending on executor/callback-group setup.
Adapter implementations must be data-race free under concurrent execution.

## Rules

1. Protect every mutable shared state with a mutex.
2. Split mutexes by responsibility when states are unrelated.
3. Do not hold adapter mutex while calling `gateway_->submit()` / `submit_request()`.
4. Do not hold adapter mutex while doing long I/O (publish, UDS send/recv, service call).
5. If multiple mutexes are needed, define and document a strict lock order.
6. Prefer lock-free snapshots:
   - copy the minimum shared state under lock,
   - release lock,
   - perform heavy computation,
   - write back results under lock.

## Recommended Mutex Split

- `status_mutex`: latest inbound status caches/timestamps.
- `policy_mutex`: command-derived mode/flag caches updated from `execute()`.
- `result_mutex`: derived diagnostic/result caches read by diagnostic callbacks.

## Deadlock Risk Checklist

- Calling `gateway_->submit()` while holding adapter mutex: high risk.
- Calling `diagnostic_updater::Updater::force_update()` while holding a state mutex that
  `update_*_diag()` also takes: medium risk.
- Nested locks without fixed order: high risk.

## Current Built-in Adapters

- `DiagAdapter`:
  - `updater_mutex_`: serializes `diagnostic_updater::Updater::force_update()`.
  - `transition_mutex_`: protects the transitional-state start timestamp.
  - **Lock order**: `updater_mutex_` → `transition_mutex_`.
    `execute()` takes `updater_mutex_` and calls `force_update()`, which synchronously
    invokes `update_status()`, which takes `transition_mutex_`. The reverse order never
    occurs, so this nesting is deadlock-free. Do not introduce a path that takes
    `transition_mutex_` first and then `updater_mutex_`.
- `SubSystemAdapter`:
  - `state_mutex_`: callback-owned state (`last_command_mode_request_`, availability timeout state).

These patterns should be reused by plugin adapter authors.
