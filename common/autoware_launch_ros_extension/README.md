# autoware_launch_ros_extension

This package provides following features for Autoware as an extension to [launch](https://github.com/ros2/launch) and [launch_ros](https://github.com/ros2/launch_ros).

- Substitutions
  - CurrentRosNamespace

## CurrentRosNamespace

The current-ros-namespace substitution returns the current ROS namespace set by the push-ros-namespace action.
This is useful when you need to specify the full name of a topic or node name.

### Sample input

```xml
<launch>
    <push-ros-namespace namespace="foo"/>
    <log message="$(current-ros-namespace)/test"/>
    <push-ros-namespace namespace="bar"/>
    <log message="$(current-ros-namespace)/test"/>
</launch>
```

### Sample output

```bash
[INFO] [launch.user]: /foo/test
[INFO] [launch.user]: /foo/bar/test
```
