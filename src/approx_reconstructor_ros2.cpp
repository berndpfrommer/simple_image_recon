// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "simple_image_recon/approx_reconstructor_ros2.hpp"

#include <event_array_msgs/msg/event_array.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "simple_image_recon/check_endian.hpp"

namespace simple_image_recon
{
ApproxReconstructor::ApproxReconstructor(
  FrameHandler * fh, const std::string & topic, int cutoffNumEvents, double fps,
  double fillRatio, int tileSize)
: frameHandler_(fh),
  topic_(topic),
  cutoffNumEvents_(cutoffNumEvents),
  fillRatio_(fillRatio),
  tileSize_(tileSize)
{
  sliceInterval_ = static_cast<uint64_t>(1000000000 / std::abs(fps));
  imageMsgTemplate_.height = 0;
}

void ApproxReconstructor::processMsg(EventArray::ConstSharedPtr msg)
{
  if (imageMsgTemplate_.height == 0) {
    imageMsgTemplate_.header = msg->header;
    imageMsgTemplate_.width = msg->width;
    imageMsgTemplate_.height = msg->height;
    imageMsgTemplate_.encoding = "mono8";
    imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
    imageMsgTemplate_.step = imageMsgTemplate_.width;
    imageMsgTemplate_.data.resize(msg->width * msg->height, 0);
    FirstMsgProcessor firstMsgProcessor;
    event_array_codecs::DecoderFactory<FirstMsgProcessor> firstFactory;
    auto firstDecoder =
      firstFactory.getInstance(msg->encoding, msg->width, msg->height);
    firstDecoder->decode(
      &(msg->events[0]), msg->events.size(), &firstMsgProcessor);
    const uint64_t t0 = firstMsgProcessor.getFirstTimeStamp();
    nextFrameTime_ = (t0 / sliceInterval_) * sliceInterval_;
    simpleReconstructor_.initialize(
      msg->width, msg->height,
      static_cast<uint32_t>(std::abs(cutoffNumEvents_)), tileSize_, fillRatio_);
    decoder_ =
      decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
    if (!decoder_) {
      std::cerr << "invalid encoding: " << msg->encoding << std::endl;
      throw(std::runtime_error("invalid encoding!"));
    }
  }
  decoder_->decode(&(msg->events[0]), msg->events.size(), this);
}

void ApproxReconstructor::emitFrame()
{
  // TODO(Bernd) avoid unnecessary copy here
  simpleReconstructor_.getImage(
    &imageMsgTemplate_.data[0], imageMsgTemplate_.step);
  auto msg = std::make_unique<sensor_msgs::msg::Image>(imageMsgTemplate_);
  msg->header.stamp = rclcpp::Time(nextFrameTime_, RCL_SYSTEM_TIME);
  frameHandler_->frame(std::move(msg), topic_);
}

}  // namespace simple_image_recon
