# VadModel Design

- code: [vad_model.hpp](../include/autoware/tensorrt_vad/vad_model.hpp)

## Responsibilities

- Responsible for CUDA-based processing.
- Receives `VadInputData`, executes inference, and returns `VadOutputData`.
- Preprocesses images using CUDA and transfers data from Host (CPU) to Device (GPU).
- Transfers data from Device (GPU) to Host (CPU) and postprocesses using CUDA.

## Processing Flowchart

```mermaid
flowchart TD
    Start([VadNode calls VadModel::infer]) --> VadInputDataBox((VadInputData))
    VadInputDataBox --> LoadInputs
    
    subgraph InferScope["VadModel::infer()"]
        LoadInputs[load_inputs: Preprocess input data and transfer from CPU to GPU]
        LoadInputs --> Enqueue[enqueue: Execute TensorRT inference]
        Enqueue --> SavePrevBev[save_prev_bev: Transfer prev_bev from GPU to CPU for next frame]
        SavePrevBev --> Postprocess[postprocess: Postprocess output and transfer from GPU to CPU]
        
        Postprocess --> CheckFirstFrame{Is first frame?}
        
        CheckFirstFrame -->|Yes| ReleaseNetwork[release_network: Release first-frame-only ONNX]
        ReleaseNetwork --> LoadHead[load_head: Load head ONNX that uses previous frame's BEV features]
        LoadHead --> ReturnText[return]
        
        CheckFirstFrame -->|No| ReturnText
    end
    
    ReturnText --> VadOutputDataBox((VadOutputData))
    

    
    style InferScope fill:#e1f5fe,stroke:#0288d1,stroke-width:2px,color:#000000
    style CheckFirstFrame fill:#fff3e0,stroke:#f57c00,stroke-width:2px,color:#000000
    style ReleaseNetwork fill:#ffebee,stroke:#d32f2f,stroke-width:2px,color:#000000
    style LoadHead fill:#e8f5e8,stroke:#388e3c,stroke-width:2px,color:#000000
    
    %% Links to source code files
    click Start "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_node.hpp" "VadNode header file"
    click VadInputDataBox "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadInputData definition"
    click LoadInputs "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click Enqueue "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click SavePrevBev "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click Postprocess "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click ReleaseNetwork "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click LoadHead "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/vad_model.hpp" "VadModel implementation"
    click VadOutputDataBox "https://github.com/autowarefoundation/autoware_universe/tree/main/planning/autoware_tensorrt_vad/include/autoware/tensorrt_vad/data_types.hpp" "VadOutputData definition"
```

### Function Roles

### API functions(public)

- [`infer()`](../include/autoware/tensorrt_vad/vad_model.hpp): Receives `VadInputData`, preprocesses images using CUDA, transfers data from Host (CPU) to Device (GPU), executes TensorRT engine, transfers data from Device (GPU) to Host (CPU), postprocesses using CUDA, and returns `VadOutputData`. Called from `VadNode`. Returns `std::nullopt` if inference fails.

### Internal functions(private)

- [`load_head()`](../include/autoware/tensorrt_vad/vad_model.hpp): VAD takes previous frame's BEV features (`prev_bev`) as input. However, for the first frame, no previous BEV features exist, so it uses a special ONNX designed only for the first frame execution. After the first frame inference is completed, `load_head()` loads the head ONNX that uses previous frame's BEV features as input.

- [`save_prev_bev()`](../include/autoware/tensorrt_vad/vad_model.hpp): Transfers `prev_bev` from Device (GPU) to Host (CPU) and saves it for the next frame's inference. The return value is stored in the `VadModel` member variable `saved_prev_bev_`.

## TODO

- Use `prev_bev` without transferring from Device (GPU) to Host (CPU).
