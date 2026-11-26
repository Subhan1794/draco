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
#include "draco/compression/point_cloud/point_cloud_sequential_decoder.h"
#include "draco/compression/attributes/linear_sequencer.h"
#include "draco/compression/attributes/sequential_attribute_decoders_controller.h"

namespace draco {

bool PointCloudSequentialDecoder::DecodeGeometryData() {
  int32_t num_points;
  if (!buffer()->Decode(&num_points)) {
    return false;
  }
  if (num_points < 0) {
    return false;
  }
  // The header num_points is attacker-controlled and is otherwise
  // unbounded: a few-hundred-byte file can claim 2,147,483,647 points,
  // and the decoders below would then attempt multi-gigabyte
  // allocations (8.6 GB for the point sequence alone) and abort the
  // process. Reject absurd claims up front with a generous absolute
  // cap. This is deliberately NOT a bound against the remaining input
  // size: entropy coding can legitimately encode many points in few
  // bytes, so a size-ratio bound would false-positive on valid
  // highly-compressible files. 100M points is far above any realistic
  // single point cloud while keeping the worst-case sequence
  // allocation at 400 MB; allocation failures below the cap are still
  // handled gracefully by the try/catch guards in LinearSequencer and
  // DataBuffer.
  constexpr int32_t kMaxNumPoints = 100000000;
  if (num_points > kMaxNumPoints) {
    return false;
  }
  point_cloud()->set_num_points(num_points);
  return true;
}

bool PointCloudSequentialDecoder::CreateAttributesDecoder(
    int32_t att_decoder_id) {
  // Always create the basic attribute decoder.
  return SetAttributesDecoder(
      att_decoder_id,
      std::unique_ptr<AttributesDecoder>(
          new SequentialAttributeDecodersController(
              std::unique_ptr<PointsSequencer>(
                  new LinearSequencer(point_cloud()->num_points())))));
}

}  // namespace draco
