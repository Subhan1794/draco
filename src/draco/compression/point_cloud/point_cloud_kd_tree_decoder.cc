// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "draco/compression/point_cloud/point_cloud_kd_tree_decoder.h"
#include "draco/compression/attributes/kd_tree_attributes_decoder.h"

namespace draco {

bool PointCloudKdTreeDecoder::DecodeGeometryData() {
  int32_t num_points;
  if (!buffer()->Decode(&num_points)) {
    return false;
  }
  if (num_points < 0) {
    return false;
  }
  // Same generous absolute cap as PointCloudSequentialDecoder: the
  // header num_points is attacker-controlled and otherwise unbounded
  // (a small file claiming 2,147,483,647 points drove a ~25.7 GB
  // attribute-buffer allocation and aborted the process). Not a
  // bound against remaining input size, which would false-positive on
  // valid highly-compressible files.
  constexpr int32_t kMaxNumPoints = 100000000;
  if (num_points > kMaxNumPoints) {
    return false;
  }
  point_cloud()->set_num_points(num_points);
  return true;
}

bool PointCloudKdTreeDecoder::CreateAttributesDecoder(int32_t att_decoder_id) {
  // Always create the basic attribute decoder.
  return SetAttributesDecoder(
      att_decoder_id,
      std::unique_ptr<AttributesDecoder>(new KdTreeAttributesDecoder()));
}

}  // namespace draco
