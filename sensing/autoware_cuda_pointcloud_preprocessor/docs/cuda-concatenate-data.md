# cuda_concatenate_and_time_synchronize_node

This package is a cuda accelerated version of the one available in [autoware_cuda_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor).
As this node is templated, the overall design, algorithm, inputs, and outputs are the same.

The only change, corresponds to the pointcloud topics, which instead of using the standard `sensor_msgs::msg::PointCloud2` message type, they use the `cuda_blackboard` mechanism.
