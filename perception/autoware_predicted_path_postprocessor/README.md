# autoware_predicted_path_postprocessor

## Purpose

The `autoware_predicted_path_postprocessor` performs post-processing on predicted paths.

## Inner-workings / Algorithms

The following processors are supported:

- [RefineBySpeed](./docs/refine_by_speed.md)
  - Refine the paths of objects based on their current speed.
- [RefinePenetrationByStaticObjects](./docs/refine_penetration_by_static_objects.md)
  - Refine the penetrated paths of objects based on the other static objects.

## Inputs / Outputs

### Input

| Name                  | Type                                              | Description            |
| --------------------- | ------------------------------------------------- | ---------------------- |
| `~/input/objects`     | `autoware_perception_msgs::msg::PredictedObjects` | Predicted objects      |
| `~/input/lanelet_map` | `autoware_msgs::msg::LaneletMapBin`               | [OPTIONAL] Lanelet map |

### Output

| Name               | Type                                              | Description       |
| ------------------ | ------------------------------------------------- | ----------------- |
| `~/output/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Processed objects |

## Quick Start

### Launch ROS 2 Node

To use this package, you can launch it with the following command:

```bash
ros2 launch autoware_predicted_path_postprocessor predicted_path_postprocessor.launch.xml
```

### Leverage Processor in Your Codebase

You can leverage the processor in your codebase by including the appropriate headers and using the `ComposableProcessor` class:

```cpp
#include <autoware/predicted_path_postprocessor/processor/composable.hpp>
#include <autoware/predicted_path_postprocessor/processor/interface.hpp>

class SomeNode final : public rclcpp::Node
{
public:
  explicit SomeNode(const rclcpp::NodeOptions& options)
    : Node("some_node", options)
  {
    // Initialize your node here
    auto processors = declare_parameter<std::vector<std::string>>("processors");
    context_ = std::make_unique<autoware::predicted_path_postprocessor::processor::Context>();
    processor_ = std::make_unique<autoware::predicted_path_postprocessor::processor::ComposableProcessor>(this, processors);
  }

private:
  void callback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & msg)
  {
    auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>(*msg);

    // update the context with the predicted objects
    context_->update(objects);

    // process the predicted objects using the processor
    const auto result = processor_->process(objects, context_);
    if (result) {
      const auto processed_objects = result.ok();
      // do something with the processed objects
      // ...
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to process predicted objects");
    }
  }

  std::unique_ptr<autoware::predicted_path_postprocessor::processor::Context> context_;
  std::unique_ptr<autoware::predicted_path_postprocessor::processor::ComposableProcessor> processor_;
};
```

## How to Add New Processor

Processors in this package should follow a structured naming convention as below:

| Class Name   | String Identifier | Roles                                                       |
| ------------ | ----------------- | ----------------------------------------------------------- |
| `RefineBy**` | `refine_by_**`    | Modify or improve existing paths based on specific criteria |
| `FilterBy**` | `filter_by_**`    | Remove or exclude paths that don't meet specific criteria   |

As an example, let's see how to add a new processor by using a processor called `FilterBySomething`.

1. Create a new processor class that inherits from `ProcessorInterface`:

   ```cpp
   class FilterBySomething final : public ProcessorInterface
   {
     public:
       FilterBySomething(rclcpp::Node * node_ptr, const std::string & processor_name)
       : ProcessorInterface(processor_name)
       {
         // Load parameters
         double_param_ = node_ptr->declare_parameter<double>(processor_name + ".double_param");
         string_param_ = node_ptr->declare_parameter<std::string>(processor_name + ".string_param");
       }

    private:
       result_type check_context(const Context & context) const noexcept override
       {
         // Override if necessary!!
         // ...Check context if it contains the required information
         return make_ok<error_type>();
       }

       result_type process(target_type & target, const Context & context) override
       {
         // ...Execute processor specific logic
         return make_ok<error_type>();
       }

    private:
      double double_param_;
      std::string string_param_;
   };
   ```

2. Register the new processor in `build_processors(...)` function:

   ```cpp
   std::vector<ProcessorInterface::UniquePtr> build_processors(rclcpp::Node * node_ptr, const std::string & processor_name)
   {
     std::vector<ProcessorInterface::UniquePtr> outputs;
     for (const auto & name : processor_names) {
       if ( /* ... */ ) {
         // ...
       } else if (name == "filter_by_something") {
         outputs.push_back(std::make_unique<FilterBySomething>(node_ptr, name));
       }
     }
     return outputs;
   }
   ```

3. Add parameter to the `config/predicted_path_postprocessor.param.yaml`:

   The parameters must be grouped under the processor's string identifier.
   The processors specified in the `processors` array are launched in runtime.

   ```yaml
   /**:
     ros__parameters:
       processors: [filter_by_something]
       # --- FilterBySomething ---
       filter_by_something:
         double_param: 100.0
         string_param: I'm a processor!!
       # --- Parameters for other processors ---
       # ...
   ```
