/**
 * Host-side helpers to build OpenCV costmaps for RacerCostMap.
 */
#pragma once

#include <mppi/path/path_reference_generator.hpp>

#include <opencv2/opencv.hpp>

#include <vector>

namespace mppi
{
namespace cost
{

struct RacerCostmapObstacle
{
  float ox = 0.0F;
  float oy = 0.0F;
  float r = 0.0F;

  RacerCostmapObstacle() = default;

  RacerCostmapObstacle(const float ox_in, const float oy_in, const float r_in) : ox(ox_in), oy(oy_in), r(r_in)
  {
  }
};

inline void updateRacerCostmap(cv::Mat& costmap_img, const std::vector<mppi::path::PathReferenceSample>& ref,
                               const std::vector<RacerCostmapObstacle>& obstacles, const int width, const int height,
                               const float ppm, const float x_min, const float y_min, std::vector<float4>& cost_map_gpu_data)
{
  const auto world_to_map = [&](const float x, const float y) {
    return cv::Point(static_cast<int>((x - x_min) * ppm), static_cast<int>((y - y_min) * ppm));
  };

  costmap_img = cv::Mat::ones(height, width, CV_32FC1);
  cv::Mat center_img = cv::Mat::ones(height, width, CV_8UC1) * 255;

  if (ref.size() >= 2)
  {
    for (size_t i = 0; i + 1 < ref.size(); ++i)
    {
      cv::line(center_img, world_to_map(ref[i].x, ref[i].y), world_to_map(ref[i + 1].x, ref[i + 1].y), cv::Scalar(0),
               1);
    }
  }

  cv::Mat dist_img;
  cv::distanceTransform(center_img, dist_img, cv::DIST_L2, 3);
  dist_img.convertTo(costmap_img, CV_32FC1, 1.0 / 25.0);
  cv::threshold(costmap_img, costmap_img, 0.75, 0.75, cv::THRESH_TRUNC);

  for (const RacerCostmapObstacle& obs : obstacles)
  {
    cv::circle(costmap_img, world_to_map(obs.ox, obs.oy), static_cast<int>(obs.r * ppm), cv::Scalar(1.0), -1);
  }

  cv::GaussianBlur(costmap_img, costmap_img, cv::Size(5, 5), 1.0);

  const int n = width * height;
  cost_map_gpu_data.resize(static_cast<size_t>(n));
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      cost_map_gpu_data[static_cast<size_t>(i * width + j)] = make_float4(costmap_img.at<float>(i, j), 0.0F, 0.0F, 0.0F);
    }
  }
}

}  // namespace cost
}  // namespace mppi
