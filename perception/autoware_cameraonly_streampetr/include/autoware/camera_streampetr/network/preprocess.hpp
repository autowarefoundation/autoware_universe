#ifndef AUTOWARE__CAMERA_STREAMPETR_PREPROCESS_HPP__
#define AUTOWARE__CAMERA_STREAMPETR_PREPROCESS_HPP__

#include <cstdint>

namespace autoware::camera_streampetr
{
    cudaError_t resizeAndExtractRoi_launch(
    const std::uint8_t * input_img, float * output_img, 
    int camera_offset,         // Camera offset in the input image
    int H, int W,              // Original image dimensions
    int H2, int W2,            // Resized image dimensions
    int H3, int W3,            // ROI dimensions
    int y_start, int x_start,  // ROI top-left coordinates in resized image
    const float * channel_wise_mean,
    const float * channel_wise_std,
    cudaStream_t stream
    );
}

#endif  // AUTOWARE__CAMERA_STREAMPETR_PREPROCESS_HPP__