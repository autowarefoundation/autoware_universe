# VadModel Design

- code: [vad_model.hpp](../include/autoware/tensorrt_vad/vad_model.hpp)

## Responsibilities

- Responsible for CUDA-based processing.
- Receives `VadInputData`, executes inference, and returns `VadOutputData`.
- Preprocesses images using CUDA and transfers data from Host (CPU) to Device (GPU).
- Transfers data from Device (GPU) to Host (CPU) and postprocesses using CUDA.

## Processing Flowchart

### Function Roles

### API functions(public)

- [`infer()`](../include/autoware/tensorrt_vad/vad_model.hpp): Receives `VadInputData`, preprocesses images using CUDA, transfers data from Host (CPU) to Device (GPU), executes TensorRT engine, transfers data from Device (GPU) to Host (CPU), postprocesses using CUDA, and returns `VadOutputData`. Called from `VadNode`. Returns `std::nullopt` if inference fails.

### Internal functions(private)

- [`load_head()`](../include/autoware/tensorrt_vad/vad_model.hpp): VAD takes previous frame's BEV features (`prev_bev`) as input. However, for the first frame, no previous BEV features exist, so it uses a special ONNX designed only for the first frame execution. After the first frame inference is completed, `load_head()` loads the head ONNX that uses previous frame's BEV features as input.

- [`save_prev_bev()`](../include/autoware/tensorrt_vad/vad_model.hpp): Transfers `prev_bev` from Device (GPU) to Host (CPU) and saves it for the next frame's inference. The return value is stored in the `VadModel` member variable `saved_prev_bev_`.

## TODO

- Use `prev_bev` without transferring from Device (GPU) to Host (CPU).
