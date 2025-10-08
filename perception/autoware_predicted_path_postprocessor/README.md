# autoware_predicted_path_postprocessor

## Purpose

The `autoware_predicted_path_postprocessor` performs post-processing on predicted paths.

## Inner-workings / Algorithms

The following processors are supported:

- [RefineBySpeed](./docs/refine_by_speed.md)
  - Refine the paths of objects based on their current speed.

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

## How to Add New Processor

Processors in this package should follow a structured naming convention as below:

| Class Name   | String Identifier | Roles                                                       |
| ------------ | ----------------- | ----------------------------------------------------------- |
| `RefineBy**` | `refine_by_**`    | Modify or improve existing paths based on specific criteria |
| `FilterBy**` | `filter_by_**`    | Remove or exclude paths that don't meet specific criteria   |

As an example, let's see how to add a new processor by using a processor called `FilterBySomething`.

1. Create a new processor class that inherits from `ProcessorInterface`:

   ```c++:processor/filter_by_something.hpp
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
       result_type check_context(const Context & context)
       {
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

   ```c++:processor/builder.hpp
   std::vector<ProcessorInterface::UniquePtr> build_processors(rclcpp::Node * node_ptr, const std::string & processor_name)
   {
     std::vector<ProcessorInterface::UniquePtr> outputs;
     for (const auto & name : processor_names) {
       if ( /* ... */) {
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

   ```yaml:config/predicted_path_postprocessor.param.yaml
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
