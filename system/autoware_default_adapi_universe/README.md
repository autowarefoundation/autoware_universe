# autoware_default_adapi_universe

## Notes

Components that relay services must be executed by the Multi-Threaded Executor.

## Features

This package is a default implementation AD API.

- [autoware state (backward compatibility)](document/autoware-state.md)
- [fail-safe](document/fail-safe.md)
- [interface](document/interface.md)
- [localization](document/localization.md)
- [motion](document/motion.md)
- [operation mode](document/operation-mode.md)
- [routing](document/routing.md)

## Interface

- [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
- [Parameters](./document/parameters.md)
- [Diagnostics](./document/diagnostics.md)

## Web server script

This is a sample to call API using HTTP.

## Guide message script

This script is no longer supported, please use rqt_diagnostic_graph_monitor instead.
This tool is available in [autoware_tools](https://github.com/autowarefoundation/autoware_tools).

```bash
ros2 run rqt_diagnostic_graph_monitor rqt_diagnostic_graph_monitor
```
