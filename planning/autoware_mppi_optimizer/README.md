# Autoware MPPI Optimizer

The `autoware_mppi_optimizer` package optimizes trajectories using Model Predictive Path Integral (MPPI) control.

## Overview

This package depends on the external [mppi_generic_vendor](https://github.com/autowarefoundation/mppi_generic_vendor) package (`MPPI-Generic`) and ships first-order Dubins path-tracking extensions under `include/mppi/`. A CUDA library interface (`FirstOrderDubinsMppiInterface`) wraps the two-lane double-park MPPI example for use from C++ / ROS nodes.

### Layout

```text
autoware_mppi_optimizer/
├── include/mppi/         # First-order Dubins dynamics, cost, path utilities
├── include/autoware/mppi_optimizer/
│   └── first_order_dubins_mppi_interface.hpp
└── src/first_order_dubins/
    └── first_order_dubins_mppi_interface.cu
```

### Requirements

- CUDA Toolkit (curand, cufft)
- Eigen3

## Inputs / Outputs

| Topic                 | Type                                    | Description          |
| --------------------- | --------------------------------------- | -------------------- |
| `~/input/trajectory`  | `autoware_planning_msgs/msg/Trajectory` | Reference trajectory |
| `~/input/odometry`    | `nav_msgs/msg/Odometry`                 | Current ego state    |
| `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Optimized trajectory |

## Launch

```bash
ros2 launch autoware_mppi_optimizer mppi_optimizer.launch.xml
```
