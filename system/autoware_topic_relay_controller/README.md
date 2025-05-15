# topic_relay_controller

## Purpose

The node subscribes to a specified topic, remaps it, and republishes it. Additionally, it has the capability to continue publishing the last received value if the subscription stops.

## Inputs / Outputs

### Input

| Name         | Type                       | Description                                                          |
| ------------ | -------------------------- | -------------------------------------------------------------------- |
| `/<topic>`   | `<specified message type>` | Topic to be subscribed, as defined by the `topic` parameter.         |
| `/tf`        | `tf2_msgs::msg::TFMessage` | (Optional) If the topic is `/tf`, used for transform message relay.  |
| `/tf_static` | `tf2_msgs::msg::TFMessage` | (Optional) If the topic is `/tf_static`, used for static transforms. |

### Output

| Name             | Type                       | Description                                                                   |
| ---------------- | -------------------------- | ----------------------------------------------------------------------------- |
| `/<remap_topic>` | `<specified message type>` | Republished topic after remapping, as defined by the `remap_topic` parameter. |

## Parameters

| Variable               | Type    | Description                                                                                          |
| ---------------------- | ------- | ---------------------------------------------------------------------------------------------------- |
| topic                  | string  | The name of the input topic to subscribe to                                                          |
| remap_topic            | string  | The name of the output topic to publish to                                                           |
| topic_type             | string  | The type of messages being relayed                                                                   |
| qos                    | integer | QoS profile to use for subscriptions and publications (default: `1`)                                 |
| transient_local        | boolean | Enables transient local QoS for subscribers (default: `false`)                                       |
| best_effort            | boolean | Enables best-effort QoS for subscribers (default: `false`)                                           |
| enable_relay_control   | boolean | Allows dynamic relay control via a service (default: `true`)                                         |
| srv_name               | string  | The service name for relay control when `enable_relay_control` is `true`                             |
| enable_keep_publishing | boolean | Keeps publishing the last received topic value when not subscribed (default: `false`)                |
| update_rate            | integer | The rate (Hz) for publishing the last topic value when `enable_keep_publishing` is `true` (optional) |
| frame_id               | string  | Frame ID for transform messages when subscribing to `/tf` or `/tf_static` (optional)                 |
| child_frame_id         | string  | Child frame ID for transform messages when subscribing to `/tf` or `/tf_static` (optional)           |

## Assumptions / Known limits

- The node assumes that the specified `topic` and `remap_topic` are valid and accessible within the ROS 2 environment.
- If `enable_keep_publishing` is `true`, the node continuously republishes the last received value even if no new messages are being received.
- For `/tf` and `/tf_static`, additional parameters like `frame_id` and `child_frame_id` are required for selective transformation relays.
- QoS settings must be carefully chosen to match the requirements of the subscribed and published topics.
