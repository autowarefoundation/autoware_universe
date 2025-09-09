#ifndef AUTOWARE__CUDA_POINT_TYPES_HPP_
#define AUTOWARE__CUDA_POINT_TYPES_HPP_

namespace autoware::cuda
{

// TODO: this is not a 
struct alignas(16) PointXYZ {
    float x, y, z;
};

struct alignas(16) PointXYZI {
    float x, y, z, intensity;
};

}

#endif