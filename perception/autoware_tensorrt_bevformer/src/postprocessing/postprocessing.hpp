// Copyright 2025 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include "ros_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

struct PredictionDict
{
  std::vector<std::vector<double>> bboxes;  // For 2D array of bounding boxes
  std::vector<double> scores;
  std::vector<int> labels;
};
using Box3D = autoware::tensorrt_bevformer::Box3D;
using PredsDict = std::vector<PredictionDict>;

struct Detection
{
  std::vector<std::vector<double>> bboxes;
  std::vector<double> scores;
  std::vector<int> labels;
};

using ResultList = std::vector<Detection>;

class PostProcessor
{
public:
  PostProcessor(
    int nb_dec, int num_query, int cls_out_channels, int bbox_dims, int bev_h, int bev_w,
    float score_thr, int max_num, const std::vector<double> & pc_range,
    const std::vector<double> & post_center_range);
  PostProcessor();
  ~PostProcessor();

  /**
   * Process the reshaped TensorRT outputs to create detection results
   * @param reshapedOutputs Map of tensor names to their data arrays
   * @return Vector of maps containing detection results
   */
  std::vector<Box3D> post_process(
    const std::map<std::string, std::vector<double>> & reshapedOutputs) const;

  int nb_dec;
  int num_query;
  int cls_out_channels;
  int bbox_dims;
  int bev_h;
  int bev_w;
  double score_thr;
  int max_num;
  std::vector<double> pc_range_;
  std::vector<double> post_center_range_;

private:
};

// Function to denormalize bounding box coordinates
std::vector<std::vector<double>> denormalize_bbox(
  const std::vector<std::vector<double>> & bbox_preds);
// Function to decode a single prediction
PredictionDict decode_single(
  const std::vector<double> & cls_scores, const std::vector<double> & bbox_preds,
  const PostProcessor & processor);

// Function to decode all predictions
PredsDict decode(
  const std::map<std::string, std::vector<double>> & preds_dicts, const PostProcessor & processor);

// Function to get bounding boxes
ResultList GetBboxes(
  const std::map<std::string, std::vector<double>> & processDict, const PostProcessor & processor);
