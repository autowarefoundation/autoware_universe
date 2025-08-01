# Scene Understanding TensorRT ROS2 Package

This package provides a ROS2 node that performs scene understanding using TensorRT for efficient inference on NVIDIA GPUs.

## Features

- Real-time scene understanding from camera images
- TensorRT optimization for fast inference
- Support for FP16 and INT8 precision
- Component-based architecture for flexibility
- Comprehensive scene analysis including:
  - Road conditions (geometry, surface, type)
  - Traffic conditions (density, control signals)
  - Environmental factors (weather, lighting, visibility)
  - Special vehicles and vulnerable road users detection

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- NVIDIA GPU with CUDA support
- TensorRT 8.x
- OpenCV 4.x
- CUDA 11.x or 12.x

## Installation

- Install TensorRT (if not already installed):

```bash
# Follow NVIDIA's official TensorRT installation guide
# https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html
```

- Clone and build the package:

```bash
cd ~/ros2_ws/src
git clone <repository_url> scene_understanding_tensorrt
cd ~/ros2_ws
colcon build --packages-select scene_understanding_tensorrt
```

## Usage

### Running the node

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Run with default parameters
ros2 run scene_understanding_tensorrt scene_understanding_node

# Or use the launch file
ros2 launch scene_understanding_tensorrt scene_understanding.launch.py
```

### Launch file parameters

```bash
ros2 launch scene_understanding_tensorrt scene_understanding.launch.py \
  onnx_model_path:=/path/to/model.onnx \
  input_image_topic:=/camera/image_raw \
  output_topic:=/scene_understanding \
  use_fp16:=true \
  use_int8:=false
```

### Topics

**Subscribed Topics:**

- `~image_raw` (sensor_msgs/Image): Input camera image

**Published Topics:**

- `~scene_understanding` (scene_understanding_tensorrt/SceneUnderstanding): Scene analysis results

### Parameters

- `onnx_model_path` (string): Path to ONNX model file
- `input_image_topic` (string): Input image topic name
- `output_topic` (string): Output result topic name
- `use_fp16` (bool): Enable FP16 precision (default: true)
- `use_int8` (bool): Enable INT8 precision (default: false)
- `max_batch_size` (int): Maximum batch size (default: 1)

## Message Format

The `SceneUnderstanding` message contains:

**Exclusive Categories (string):**

- `intersection`: Type of intersection
- `lane_position`: Current lane position
- `lighting`: Lighting conditions
- `road_geometry`: Road shape
- `road_surface`: Surface condition
- `road_type`: Type of road
- `traffic_density`: Traffic level
- `visibility`: Visibility conditions
- `weather`: Weather conditions

**Binary Categories (float32, 0-1 probability):**

- Road features (construction zone, crosswalk, merge area, railway crossing)
- Special vehicles (emergency, heavy, public transport)
- Traffic control (no controls, stop sign, traffic light, yield sign)
- Vulnerable road users (animals, cyclists, pedestrians)

## Performance Notes

- First inference may be slower due to TensorRT optimization
- FP16 mode provides good balance between speed and accuracy
- INT8 mode requires calibration for best results
- Typical inference time: 5-20ms on modern NVIDIA GPUs

## Troubleshooting

1. **TensorRT not found**: Set `TensorRT_ROOT` in CMakeLists.txt or as environment variable
2. **CUDA errors**: Ensure NVIDIA drivers and CUDA are properly installed
3. **Model loading fails**: Verify ONNX model path and compatibility

## License

Apache-2.0
