# autoware_agnocast_wrapper

The purpose of this package is to integrate Agnocast, a zero-copy middleware, into each topic in Autoware with minimal side effects. Agnocast is a library designed to work alongside ROS 2, enabling true zero-copy publish/subscribe communication for all ROS 2 message types, including unsized message types.

- Agnocast Repository: <https://github.com/tier4/agnocast>
- Discussion on Agnocast Integration into Autoware: <https://github.com/orgs/autowarefoundation/discussions/5835>

This package provides macros that wrap functions for publish/subscribe operations and smart pointer types for handling ROS 2 messages. When Autoware is built using the default build command, Agnocast is **not enabled**. However, setting the environment variable `ENABLE_AGNOCAST=1` enables Agnocast and results in a build that includes its integration. This design ensures backward compatibility for users who are unaware of Agnocast, minimizing disruption.

## How to Use the Macros in This Package

You can immediately understand how to use the macros just by looking at `autoware_agnocast_wrapper.hpp`. A typical callback and publisher setup looks like this:

```cpp
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>

pub_output_ = AUTOWARE_CREATE_PUBLISHER3(
  PointCloud2,
  "output",
  rclcpp::SensorDataQoS().keep_last(max_queue_size_),
  pub_options
);

void onPointCloud(const AUTOWARE_MESSAGE_PTR(const PointCloud2) input_msg) {
  auto output = ALLOCATE_OUTPUT_MESSAGE(pub_output_);
  ...
  pub_output_->publish(std::move(output));
}
```

To use the macros provided by this package in your own package, include the following lines in your `CMakeLists.txt`:

```cmake
find_package(autoware_agnocast_wrapper REQUIRED)
ament_target_dependencies(target autoware_agnocast_wrapper)
target_include_directories(target ${autoware_agnocast_wrapper_INCLUDE_DIRS})
autoware_agnocast_wrapper_setup(target)
```

## How to Enable/Disable Agnocast on Build

To build Autoware **with** Agnocast:

```bash
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To build Autoware **without** Agnocast (default behavior):

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To explicitly **disable** Agnocast when it has been previously enabled:

```bash
unset ENABLE_AGNOCAST
# or
export ENABLE_AGNOCAST=0
```

To rebuild a specific package **without** Agnocast after it was previously built with Agnocast:

```bash
rm -Rf ./install/<package_name> ./build/<package_name>
export ENABLE_AGNOCAST=0
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select <package_name>
```

To rebuild a specific package **with** Agnocast after it was previously built without it:

```bash
rm -Rf ./install/<package_name> ./build/<package_name>
export ENABLE_AGNOCAST=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select <package_name>
```

Please note that the `ENABLE_AGNOCAST` environment variable may not behave as expected in the following scenario:

- Package A depends on build artifacts from Package B
- Both A and B were previously built with Agnocast **enabled**
- Rebuilding only Package A with `ENABLE_AGNOCAST=0` will not be sufficient, as compile options enabling Agnocast may propagate from Package B

Example:

- A = `autoware_occupancy_grid_map_outlier_filter`
- B = `autoware_pointcloud_preprocessor`

In such cases, rebuild both A and B with Agnocast **disabled** to ensure consistency. As a best practice, we recommend keeping the value of `ENABLE_AGNOCAST` consistent within a workspace to avoid unintentional mismatches and simplify build management.
