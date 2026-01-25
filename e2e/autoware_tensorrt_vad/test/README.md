# Unit Tests for autoware_tensorrt_vad

This directory contains focused unit tests for the VAD utilities.

## Test Files

### 1. `test_param_loader.cpp`

Focused coverage for:

- `utils::load_model_params()` JSON parsing
- `utils::check_model_version()` compatibility checks
- Error handling for missing files and missing/unsupported major_version

## Building Tests

```bash
# Build the package with tests
colcon build --packages-select autoware_tensorrt_vad --cmake-args -DBUILD_TESTING=ON
```

## Running Tests

```bash
# Run all tests for this package
colcon test --packages-select autoware_tensorrt_vad

# View test results
colcon test-result --verbose

# Run specific test executable
./build/autoware_tensorrt_vad/test_param_loader

# Run with gtest filters
./build/autoware_tensorrt_vad/test_param_loader --gtest_filter="ParamLoader.*"
```

## Test Coverage

To generate code coverage reports:

```bash
# Build with coverage flags
colcon build --packages-select autoware_tensorrt_vad \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage"

# Run tests
colcon test --packages-select autoware_tensorrt_vad

# Generate coverage report with lcov
lcov --capture --directory build/autoware_tensorrt_vad \
  --output-file coverage.info \
  --exclude '/usr/*' --exclude '*/test/*'

# Generate HTML report
genhtml coverage.info --output-directory coverage_html

# View in browser
firefox coverage_html/index.html
```

## What is Tested

- Happy-path parse of a minimal valid `vad-carla-tiny.param.json`
- Error on missing file
- Error on missing `major_version`
- Error on mismatched `major_version` compared to `SUPPORTED_MAJOR_VERSION`

## Known Limitations

- These tests cover only JSON parsing and version validation. They do not exercise TensorRT, ROS 2, or the runtime node logic.

## Future Improvements

- [ ] Add integration tests once lightweight model fixtures are available
- [ ] Add schema validation for optional JSON fields

## Dependencies

- `ament_cmake_gtest`
- `ament_lint_auto`

## Troubleshooting

- Ensure `BUILD_TESTING` is ON
- If tests fail, check that temporary directories are writable (used for generating test JSON files)

## References

- [Google Test Documentation](https://google.github.io/googletest/)
