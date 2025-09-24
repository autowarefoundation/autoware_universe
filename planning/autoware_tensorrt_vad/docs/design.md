# Design

This document explains the key concepts emphasized during the design of the VAD ROS Node.

- [Separation between ROS and CUDA domains](#separation-between-ros-and-cuda-domains)
  - Changes to ROS topic types do not affect CUDA implementations
  - Changes to CUDA versions or interfaces do not affect ROS Nodes
- [Separation between ONNX-dependent and ONNX-independent ROS parameters](#separation-between-onnx-dependent-and-onnx-independent-ros-parameters)
- [Extensible design for Autoware `camera_id` changes](#extensible-design-for-autoware-camera_id-changes)
  - Even if the front `camera_id` changes from `0` to `1`, it can be handled with parameter changes, and without major design changes

## Separation between ROS and CUDA domains

`autoware_tensorrt_vad` is clearly separated into the "ROS/Autoware domain"(`VadNode`) and the "CUDA inference processing domain"(`VadModel`).

- **ROS side responsibilities**:

  - ROS topic subscription/publication
    - Topic drop and sync verification
  - Integration with Autoware (subscription/publication)

- **Interface responsibilities**:

  - Input side
    - Coordinate transformation
    - Conversion from ROS Topics(`VadInputTopicData`) to `VadInputData`
    - Camera id mapping
  - Output side
    - Coordinate transformation
    - Conversion from `VadOutputData` to ROS Topics(`VadOutputTopicData`)

- **CUDA side responsibilities**:
  - Camera image preprocessing (CUDA preprocessing)
  - VAD inference
    - Inference from `VadInputData` to `VadOutputData`
  - Output postprocessing (CUDA postprocessing)

The interface (`VadInterface`) bridges ROS side(`VadNode`) and CUDA side(`VadModel`), designed to minimize the impact of changes between them.

---

### Dependency Graph

```mermaid
graph TD
    VadNode["VadNode"]
    VadInterface["VadInterface"]
    VadInputTopicData(["VadInputTopicData"])
    VadOutputTopicData(["VadOutputTopicData"])
    VadModel["VadModel"]
    VadInputData(["VadInputData"])
    VadOutputData(["VadOutputData"])
    VadNode --> VadInputTopicData
    VadNode --> VadOutputTopicData
    VadInterface --> VadInputTopicData
    VadInterface --> VadOutputTopicData
    VadInterface --> VadInputData
    VadInterface --> VadOutputData
    VadModel --> VadInputData
    VadModel --> VadOutputData

    style VadNode fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadModel fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadInterface fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;

    style VadInputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadOutputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;

    style VadInputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadOutputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
```

- `VadInterface`: Interface between ROS side and CUDA side

- `VadInputData`, `VadOutputData`: Data structures used for inference on CUDA side(`VadModel`)

- `VadModel`: Inference model using CUDA and TensorRT

`VadInterface` depends on ROS-side `VadInputTopicData`, `VadOutputTopicData` and CUDA-side data `VadInputData` and `VadOutputData`, handling conversions between these data formats.

`VadModel` depends **only** on `VadInputData` and `VadOutputData` within the CUDA side.

This dependency structure allows `VadInterface` to function as a buffer between the ROS and CUDA domains, separating their influence and responsibility ranges to limit the impact of changes. Specifically, we expect to achieve the following states:

- When changes are needed on the ROS/Autoware side (such as ROS topic name/content changes), no changes need to be made to `VadModel`, `VadInputData`, `VadOutputData`
- When changes are needed on the CUDA/TensorRT side (such as CUDA/TensorRT version changes), no changes need to be made to `VadInputTopicData`, `VadOutputTopicData`

---

### Processing Flow Diagram

```mermaid
flowchart TD
    subgraph "ROS Domain (input)"
  VadNode_sub[VadNode::execute_inference]
  VadInputTopicData([VadInputTopicData])
        VadInterface[VadInterface]
    end

    subgraph "VAD Domain"
        VadInputData([VadInputData])
        VadModel::infer[VadModel::infer]
        VadOutputData([VadOutputData])
    end

    subgraph "ROS Domain (output)"
  VadInterface2[VadInterface]
        VadOutputTopicData([VadOutputTopicData])
        VadNode_pub[VadNode::publish]
    end

 VadNode_sub --> VadInputTopicData
 VadInputTopicData --> VadInterface

    VadInterface --> VadInputData

    VadInputData --> VadModel::infer
    VadModel::infer --> VadOutputData

    VadOutputData --> VadInterface2

    VadInterface2 --> VadOutputTopicData
    VadOutputTopicData --> VadNode_pub

    style VadNode_sub fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadInputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadInterface fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;

    style VadInputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadModel::infer fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadOutputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;

    style VadInterface2 fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;
    style VadOutputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadNode_pub fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
```

- Topic conversion and coordinate transformation should be handled by the interface (`VadInterface`).

- The inference part is included completely within `VadModel`.

---

### Expected Use Cases

#### When adding new input to VAD

- Add new input to `VadModel`. Retrain ONNX.

- Modify `VadNode` to subscribe to new topic.

- Add topic to `VadInputTopicData`.

- Modify input conversion processing in `VadInterface`.

- Add member to `VadInputData`.

## Separation between ONNX-dependent and ONNX-independent ROS parameters

- Parameters related to ONNX are added to [`ml_package_vad_tiny.param.yaml`](../config/ml_package_vad_tiny.param.yaml)
- ROS parameters that can be changed regardless of ONNX are added to [`vad_tiny.param.yaml`](../config/vad_tiny.param.yaml)
  - Object class remapping parameter is added to [`object_class_remapper.param.yaml`](../config/object_class_remapper.param.yaml)
    - Following the precedent of [`autoware_bevfusion`](../../../perception/autoware_bevfusion/README.md).
- Some parameters like `autoware_to_vad_camera_mapping` depend on both ONNX and ROS Node. If it **could** affect ONNX, it is added to [`ml_package_vad_tiny.param.yaml`](../config/ml_package_vad_tiny.param.yaml).

### Expected Use Cases

| Use Case | vad_tiny.param.yaml | ml_package_vad_tiny.param.yaml | object_class_remapper.param.yaml |
|----------|---------------------|------------------------------|--------------------------------|
| ONNX-dependent changes | Modify | Do not modify | Modify only when VAD ONNX output class definitions change |
| ONNX-independent changes | Do not modify | Modify | Modify only when object class definitions in Autoware change |

## Extensible design for Autoware `camera_id` changes

- The design is extensible for changes to `camera_id` used in Autoware.
- VAD camera image id (order) only affects `VadInterface`.
  - It does not affect `VadInputData` or `VadModel`
- Camera `camera_id` changes can be handled by modifying only `autoware_to_vad_camera_mapping`.

### Expected Use Cases

#### When camera image ID used for VAD input is changed

- Modify `autoware_to_vad_camera_mapping` in the ROS param file ([`ml_package_vad_tiny.param.yaml`](../config/ml_package_vad_tiny.param.yaml))
