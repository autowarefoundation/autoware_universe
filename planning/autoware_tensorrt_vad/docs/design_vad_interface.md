# VadInterface Design

- code: [vad_interface.cpp](../lib/vad_interface.cpp) [vad_interface.hpp](../include/autoware/tensorrt_vad/vad_interface.hpp)

## Responsibilities

-  Convert from `VadInputTopicData` to `VadInputData`, and from `VadOutputData` to `VadOutputTopicData`
   - Transform coordinate system by [`CoordinateTransformer`](../include/autoware/tensorrt_vad/coordinate_transformer.hpp)
   - Convert input data
     - Convert input images by [`InputImageConverter`](../include/autoware/tensorrt_vad/input_converter/image_converter.hpp)
     - Convert input transform matrix by [`InputTransformMatrixConverter`](../include/autoware/tensorrt_vad/input_converter/transform_matrix_converter.hpp)
     - Convert input odometry adata by [`InputCanBusConverter`](../include/autoware/tensorrt_vad/input_converter/can_bus_converter.hpp) and [`InputBEVShiftConverter`](../include/autoware/tensorrt_vad/input_converter/bev_shift_converter.hpp)
   - Convert output data
     - Convert output planning trajectory by [`OutputTrajectoryConverter`](../include/autoware/tensorrt_vad/output_converter/trajectory_converter.hpp)
     - Convert output predicted objects by [`OutputObjectsConverter`](../include/autoware/tensorrt_vad/output_converter/objects_converter.hpp)
     - Convert output map markers by [`OutputMapConverter`](../include/autoware/tensorrt_vad/output_converter/map_converter.hpp)

- Responsible for preprocessing and postprocessing that use only CPU (does not use CUDA).

## Processing Flowchart

```mermaid
flowchart TD
    VadNode["VadNode"]
    VadNode2["VadNode"]
    VadInputTopicData((VadInputTopicData))
    VadOutputData((VadOutputData))
    VadNode --> VadInputTopicData
    VadInputTopicData --> ConvertInput
    subgraph ConvertInput["VadInterface::convert_input()"]
        ImageConv
        MatrixConv
        CanBusConv
        BEVShiftConv
    end
    ConvertInput --> VadInputData((VadInputData))
    VadInputData --> VadNode2
    style VadInputData fill:#fff,stroke:#1976d2,stroke-width:2px,color:#000000
    VadNode --> VadOutputData
    VadOutputData --> ConvertOutput
    subgraph ConvertOutput["VadInterface::convert_output()"]
        TrajConv
        ObjConv
        MapConv
    end
    ConvertOutput --> VadOutputTopicData((VadOutputTopicData))
    VadOutputTopicData --> VadNode2

    subgraph VadInterface["VadInterface"]
        subgraph ConvertInput["VadInterface::convert_input()"]
            subgraph ImageConv["InputImageConverter"]
                ImageProc["process_image()"]
            end
            subgraph MatrixConv["InputTransformMatrixConverter"]
                MatrixProc["process_vad_base2img()"]
            end
            subgraph CanBusConv["InputCanBusConverter"]
                CanBusProc["process_can_bus()"]
            end
            subgraph BEVShiftConv["InputBEVShiftConverter"]
                BEVShiftProc["process_shift()"]
            end
        end
        subgraph ConvertOutput["VadInterface::convert_output()"]
            subgraph TrajConv["OutputTrajectoryConverter"]
                TrajProc["process_trajectory()"]
                TrajCandProc["process_candidate_trajectories()"]
            end
            subgraph ObjConv["OutputObjectsConverter"]
                ObjProc["process_predicted_objects()"]
            end
            subgraph MapConv["OutputMapConverter"]
                MapProc["process_map_points()"]
            end
        end
    end
    style VadOutputTopicData fill:#fff,stroke:#1976d2,stroke-width:2px,color:#000000

    style VadInputTopicData fill:#fff,stroke:#1976d2,stroke-width:2px,color:#000000
    style VadOutputData fill:#fff,stroke:#1976d2,stroke-width:2px,color:#000000
    style VadInterface fill:#e3f2fd,stroke:#1976d2,stroke-width:2px,color:#000000
    style VadNode fill:#e8f5e8,stroke:#388e3c,stroke-width:2px,color:#000000
    style VadNode2 fill:#e8f5e8,stroke:#388e3c,stroke-width:2px,color:#000000
    %% Blue style for ConvertInput/ConvertOutput subgraphs
    style ConvertInput fill:#bbdefb,stroke:#1976d2,stroke-width:2px,color:#000000
    style ConvertOutput fill:#bbdefb,stroke:#1976d2,stroke-width:2px,color:#000000

    %% Clickable links for code navigation (must be after all style statements)
    click VadNode "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/src/vad_node.cpp" "VadNode implementation"
    click VadNode2 "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/src/vad_node.cpp" "VadNode implementation"
    click VadInputTopicData "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadInputTopicData definition"
    click VadOutputData "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadOutputData definition"
    click ConvertInput "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_interface.hpp" "VadInterface::convert_input()"
    click ConvertOutput "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_interface.hpp" "VadInterface::convert_output()"
    click VadInputData "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadInputData definition"
    click VadOutputTopicData "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadOutputTopicData definition"
    click ImageProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/input_converter/image_converter.hpp" "InputImageConverter::process_image()"
    click MatrixProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/input_converter/transform_matrix_converter.hpp" "InputTransformMatrixConverter::process_vad_base2img()"
    click CanBusProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/input_converter/can_bus_converter.hpp" "InputCanBusConverter::process_can_bus()"
    click BEVShiftProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/input_converter/bev_shift_converter.hpp" "InputBEVShiftConverter::process_shift()"
    click TrajProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/output_converter/trajectory_converter.hpp" "OutputTrajectoryConverter::process_trajectory()"
    click TrajCandProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/output_converter/trajectory_converter.hpp" "OutputTrajectoryConverter::process_candidate_trajectories()"
    click ObjProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/output_converter/objects_converter.hpp" "OutputObjectsConverter::process_predicted_objects()"
    click MapProc "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/output_converter/map_converter.hpp" "OutputMapConverter::process_map_points()"
```

### Function Roles

### API functions(public)

- `VadNode` calls `convert_input()` before inference and `convert_output()` after inference.

- [`convert_input()`](../include/autoware/tensorrt_vad/vad_interface.hpp): Convert from `VadInputTopicData` to `VadInputData`
    - Image processing is handled by [`InputImageConverter::process_image()`](../include/autoware/tensorrt_vad/input_converter/image_converter.hpp)
    - Transform matrix processing is handled by [`InputTransformMatrixConverter::process_vad_base2img()`](../include/autoware/tensorrt_vad/input_converter/transform_matrix_converter.hpp)
    - Odometry data processing is handled by [`InputCanBusConverter::process_can_bus()`](../include/autoware/tensorrt_vad/input_converter/can_bus_converter.hpp) and [`InputBEVShiftConverter::process_shift()`](../include/autoware/tensorrt_vad/input_converter/bev_shift_converter.hpp)
- [`convert_output()`](../include/autoware/tensorrt_vad/vad_interface.hpp): Convert from `VadOutputData` to `VadOutputTopicData`
    - Trajectory processing is handled by [`OutputTrajectoryConverter::process_trajectory()`](../include/autoware/tensorrt_vad/output_converter/trajectory_converter.hpp) and [`OutputTrajectoryConverter::process_candidate_trajectories()`](../include/autoware/tensorrt_vad/output_converter/trajectory_converter.hpp)
    - Object processing is handled by [`OutputObjectsConverter::process_predicted_objects()`](../include/autoware/tensorrt_vad/output_converter/objects_converter.hpp)
    - Map processing is handled by [`OutputMapConverter::process_map_points()`](../include/autoware/tensorrt_vad/output_converter/map_converter.hpp)

## TODO

- The name "CanBus" is used even though data is not obtained from CAN BUS. This naming convention prioritizes conformity with [the notation used in VAD code](https://github.com/hustvl/VAD/blob/36047b6b5985e01832d8a2ecb0355d7f3c753ee1/projects/mmdet3d_plugin/datasets/nuscenes_vad_dataset.py#L1375-L1382). However, this could cause confusion, so a better name should be considered.
