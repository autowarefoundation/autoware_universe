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

#include "postprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

PostProcessor::PostProcessor(
  int nb_dec, int num_query, int cls_out_channels, int bbox_dims, int bev_h, int bev_w,
  float score_thr, int max_num, const std::vector<double> & pc_range,
  const std::vector<double> & post_center_range)
: nb_dec(nb_dec),
  num_query(num_query),
  cls_out_channels(cls_out_channels),
  bbox_dims(bbox_dims),
  bev_h(bev_h),
  bev_w(bev_w),
  score_thr(score_thr),
  max_num(max_num),
  pc_range_(pc_range),
  post_center_range_(post_center_range)
{
}

// Destructor for PostProcessor.
PostProcessor::~PostProcessor()
{
}

// Decodes predictions from model output tensors.
PredsDict decode(
  const std::map<std::string, std::vector<double>> & preds_dicts, const PostProcessor & processor)
{
  const auto & all_cls_scores_flat = preds_dicts.at("all_cls_scores");
  const auto & all_bbox_preds_flat = preds_dicts.at("all_bbox_preds");
  int decoder_cls_size = all_cls_scores_flat.size() / processor.nb_dec;
  int decoder_bbox_size = all_bbox_preds_flat.size() / processor.nb_dec;

  std::vector<double> last_cls_scores(
    all_cls_scores_flat.end() - decoder_cls_size, all_cls_scores_flat.end());

  std::vector<double> last_bbox_preds(
    all_bbox_preds_flat.end() - decoder_bbox_size, all_bbox_preds_flat.end());

  int cls_score_per_batch = processor.num_query * processor.cls_out_channels;
  int batch_size = 1;
  PredsDict prediction_list;
  for (int i = 0; i < batch_size; ++i) {
    int cls_offset = i * cls_score_per_batch;
    std::vector<double> cls_scores_per_batch_i(
      last_cls_scores.begin() + cls_offset,
      last_cls_scores.begin() + cls_offset + cls_score_per_batch);

    int bbox_pred_per_batch = processor.num_query * processor.bbox_dims;
    int bbox_offset = i * bbox_pred_per_batch;
    std::vector<double> bbox_preds_per_batch_i(
      last_bbox_preds.begin() + bbox_offset,
      last_bbox_preds.begin() + bbox_offset + bbox_pred_per_batch);
    prediction_list.push_back(
      decode_single(cls_scores_per_batch_i, bbox_preds_per_batch_i, processor));
  }
  return prediction_list;
}

// Decodes a single batch of class scores and bounding box predictions.
PredictionDict decode_single(
  const std::vector<double> & cls_scores, const std::vector<double> & bbox_preds,
  const PostProcessor & processor)
{
  int max_num = processor.max_num;
  int num_classes = processor.cls_out_channels;
  double score_thr = processor.score_thr;
  int bbox_dims = processor.bbox_dims;
  std::vector<double> post_center_range = processor.post_center_range_;
  int num_query = processor.num_query;

  PredictionDict prediction_dict;
  std::vector<double> sigmoid_cls_scores(cls_scores.size());
  for (size_t i = 0; i < cls_scores.size(); ++i) {
    sigmoid_cls_scores[i] = 1.0 / (1.0 + std::exp(-cls_scores[i]));
  }
  std::vector<std::pair<double, int>> score_index_pairs(sigmoid_cls_scores.size());
  for (size_t i = 0; i < sigmoid_cls_scores.size(); ++i) {
    score_index_pairs[i] = {sigmoid_cls_scores[i], static_cast<int>(i)};
  }
  // Partial sort is used for efficiency, only keeping the top max_num scores
  std::partial_sort(
    score_index_pairs.begin(),
    score_index_pairs.begin() + std::min(max_num, static_cast<int>(score_index_pairs.size())),
    score_index_pairs.end(), [](const auto & a, const auto & b) { return a.first > b.first; });
  int actual_num = std::min(max_num, static_cast<int>(score_index_pairs.size()));
  score_index_pairs.resize(actual_num);
  std::vector<double> scores(actual_num);
  std::vector<int> labels(actual_num);
  std::vector<int> bbox_indices(actual_num);

  for (int i = 0; i < actual_num; ++i) {
    scores[i] = score_index_pairs[i].first;
    int index = score_index_pairs[i].second;
    labels[i] = index % num_classes;
    bbox_indices[i] = index / num_classes;
  }
  std::vector<std::vector<double>> selected_bbox_preds;
  selected_bbox_preds.reserve(bbox_indices.size());
  for (int idx : bbox_indices) {
    if (idx >= 0 && idx < num_query) {
      auto start = bbox_preds.begin() + idx * bbox_dims;
      auto end = start + bbox_dims;
      selected_bbox_preds.emplace_back(start, end);
    }
  }
  std::vector<std::vector<double>> final_box_preds = denormalize_bbox(selected_bbox_preds);
  std::vector<bool> thresh_mask(scores.size(), true);
  // If score_thr is not null, apply the thresholding logic
  if (score_thr > 0.0) {
    double tmp_score = score_thr;
    std::fill(thresh_mask.begin(), thresh_mask.end(), false);
    for (size_t i = 0; i < scores.size(); ++i) {
      if (scores[i] > tmp_score) {
        thresh_mask[i] = true;
      }
    }
  }
  std::vector<bool> mask(final_box_preds.size(), true);
  for (size_t i = 0; i < final_box_preds.size(); ++i) {
    const auto & box = final_box_preds[i];
    for (int j = 0; j < 3; ++j) {
      if (box[j] < post_center_range[j] || box[j] > post_center_range[j + 3]) {
        mask[i] = false;
        break;
      }
    }
    if (score_thr > 0.0f && mask[i] && !thresh_mask[i]) {
      mask[i] = false;
    }
  }
  std::vector<std::vector<double>> bboxes3d;
  bboxes3d.reserve(mask.size());
  std::vector<double> filtered_scores;
  filtered_scores.reserve(mask.size());
  std::vector<int> filtered_labels;
  filtered_labels.reserve(mask.size());

  for (size_t i = 0; i < mask.size(); ++i) {
    if (mask[i]) {
      bboxes3d.push_back(final_box_preds[i]);
      filtered_scores.push_back(scores[i]);
      filtered_labels.push_back(labels[i]);
    }
  }
  prediction_dict.bboxes = bboxes3d;
  prediction_dict.scores = filtered_scores;
  prediction_dict.labels = filtered_labels;
  return prediction_dict;
}

// Denormalizes bounding box predictions to real-world coordinates.
std::vector<std::vector<double>> denormalize_bbox(
  const std::vector<std::vector<double>> & bbox_preds)
{
  std::vector<std::vector<double>> denorm_bboxes;

  for (const auto & bbox : bbox_preds) {
    if (bbox.size() < 8) {
      std::cerr << "Invalid bbox size: must be at least 8.\n";
      continue;
    }

    double cx = bbox[0];
    double cy = bbox[1];
    double cz = bbox[4];

    double w = std::exp(bbox[2]);
    double l = std::exp(bbox[3]);
    double h = std::exp(bbox[5]);

    double rot_sine = bbox[6];
    double rot_cosine = bbox[7];
    double rot = std::atan2(rot_sine, rot_cosine);

    std::vector<double> denorm_bbox = {cx, cy, cz, w, l, h, rot};

    // Optional velocity
    if (bbox.size() >= 10) {
      double vx = bbox[8];
      double vy = bbox[9];
      denorm_bbox.push_back(vx);
      denorm_bbox.push_back(vy);
    }

    denorm_bboxes.push_back(denorm_bbox);
  }
  return denorm_bboxes;
}

// Gets bounding boxes from processed prediction dictionary.
ResultList GetBboxes(
  const std::map<std::string, std::vector<double>> & processDict, const PostProcessor & processor)
{
  ResultList results;
  PredsDict preds_dicts;
  preds_dicts = decode(processDict, processor);
  int num_samples = preds_dicts.size();
  for (int i = 0; i < num_samples; ++i) {
    Detection detection;
    auto & preds = preds_dicts[i];
    auto & bboxes = preds.bboxes;

    for (auto & bbox : bboxes) {
      if (bbox.size() > 5) {
        bbox[2] = bbox[2] - bbox[5] * 0.5;
      }
    }

    detection.bboxes = preds.bboxes;
    detection.scores = preds.scores;
    detection.labels = preds.labels;

    results.push_back(detection);
  }
  return results;
}

// Main post-processing function to convert model outputs to 3D bounding boxes.
std::vector<Box3D> PostProcessor::post_process(
  const std::map<std::string, std::vector<double>> & reshapedOutputs) const
{
  std::vector<Box3D> ego_boxes;

  std::map<std::string, std::vector<double>> processDict;
  processDict["all_cls_scores"] = reshapedOutputs.at("outputs_classes");
  processDict["all_bbox_preds"] = reshapedOutputs.at("outputs_coords");

  ResultList results = GetBboxes(processDict, *this);

  for (const auto & detection : results) {
    for (size_t i = 0; i < detection.bboxes.size(); i++) {
      Box3D box;
      const auto & bbox = detection.bboxes[i];

      box.x = static_cast<float>(bbox[0]);
      box.y = static_cast<float>(bbox[1]);
      box.z = static_cast<float>(bbox[2]);

      box.w = static_cast<float>(bbox[3]);
      box.l = static_cast<float>(bbox[4]);
      box.h = static_cast<float>(bbox[5]);

      box.r = static_cast<float>(bbox[6]);

      if (bbox.size() > 7) {
        box.vx = static_cast<float>(bbox[7]);
      }
      if (bbox.size() > 8) {
        box.vy = static_cast<float>(bbox[8]);
      }

      box.score = static_cast<float>(detection.scores[i]);
      box.label = detection.labels[i];

      ego_boxes.push_back(box);
    }
  }
  return ego_boxes;
}
