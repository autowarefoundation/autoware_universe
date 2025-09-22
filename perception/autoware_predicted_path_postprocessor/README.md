# autoware_predicted_path_postprocessor

## Purpose

The `autoware_predicted_path_postprocessor` performs post-processing on predicted paths.

## Inner-workings / Algorithms

The following processors are supported:

- [RefineBySpeed](./docs/refine_by_speed.md)
  - Refine the paths of objects based on their current speed.

## Inputs / Outputs

### Input

| Name              | Type                                              | Description       |
| ----------------- | ------------------------------------------------- | ----------------- |
| `~/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Predicted objects |

### Output

| Name               | Type                                              | Description       |
| ------------------ | ------------------------------------------------- | ----------------- |
| `~/output/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Processed objects |

## How to Add New Processor

1. Create a new processor class that inherits from `ProcessorInterface`:

   ```c++:processors/sample_processor.hpp
   class SampleProcessor final : public ProcessorInterface
   {
     public:
       SampleProcessor(rclcpp::Node * node_ptr, const std::string & processor_name)
       : ProcessorInterface()
       {
         load_config(
           node_ptr, processor_name, [](rclcpp::Node * node_ptr, const std::string & processor_name) {
             node_ptr->declare_parameter<double>(processor_name + ".double_param", 0.0);
             node_ptr->declare_parameter<std::string>(processor_name + ".string_param", "default");
           });

         auto double_param = node_ptr->get_parameter(processor_name + ".double_param").as_double();
         auto string_param = node_ptr->get_parameter(processor_name + ".string_param").as_string();

         RCLCPP_INFO_STREAM(
           node_ptr->get_logger(), "SampleProcessor initialized!! ["
                                     << processor_name << "]: double_param=" << double_param
                                     << ", string_param=" << string_param);
       }

       void process(
         autoware_perception_msgs::msg::PredictedObject &, const Context &) override
       {
       }
   };
   ```

2. Add building logic for the processor in `build_processors(...)` function:

   ```c++:builder.hpp
   std::vector<ProcessorInterface::UniquePtr> build_processors(rclcpp::Node * node_ptr, const std::string & processor_name)
   {
     for (const auto & name : processor_names) {
       if ( /* ... */) {
         // ...
       } else if (name == "sample_processor") {
         outputs.emplace_back(std::make_unique<processors::SampleProcessor>(node_ptr, name));
       }
     }
   }
   ```

3. Add parameter file in `config/sample_processor.param.yaml`:

   ```yaml:config/sample_processor.param.yaml
   /**:
     ros__parameters:
       sample_processor:
         double_param: 100.0
         string_param: Hello, world!!
   ```
