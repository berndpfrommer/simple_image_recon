// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "simple_image_recon/approx_reconstruction_ros1.hpp"

#include <vector>

#include "simple_image_recon/check_endian.hpp"

namespace simple_image_recon
{
ApproxReconstruction::ApproxReconstruction(ros::NodeHandle & nh) : nh_(nh)
{
  const double fps = nh_.param<double>("fps", 25.0);
  sliceInterval_ = static_cast<uint64_t>(1000000000 / std::abs(fps));
  cutoffNumEvents_ = nh_.param<int>("cutoff_num_events", 30);
  fillRatio_ = nh_.param<double>("fill_ratio", 0.6);
  tileSize_ = nh_.param<int>("tile_size", 2);

  imageMsgTemplate_.height = 0;
  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise(
    "image_raw", 1,
    boost::bind(
      &ApproxReconstruction::imageConnectCallback, this,
      boost::placeholders::_1),
    boost::bind(
      &ApproxReconstruction::imageConnectCallback, this,
      boost::placeholders::_1));
}

ApproxReconstruction::~ApproxReconstruction() {}

void ApproxReconstruction::imageConnectCallback(
  const image_transport::SingleSubscriberPublisher &)
{
  if (imagePub_.getNumSubscribers()) {
    if (!isSubscribedToEvents_) {
      eventSub_ = nh_.subscribe(
        "events", 1000 /*qsize */, &ApproxReconstruction::eventMsg, this);
      isSubscribedToEvents_ = true;
      ROS_INFO_STREAM("subscribed to events!");
    }
  } else {
    if (isSubscribedToEvents_) {
      eventSub_.shutdown();  // unsubscribe
      isSubscribedToEvents_ = false;
      ROS_INFO_STREAM("unsubscribed from events!");
    }
  }
}

void ApproxReconstruction::eventMsg(const EventArray::ConstPtr & msg)
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
      ROS_ERROR_STREAM("invalid encoding: " << msg->encoding);
      throw std::runtime_error("invalid encoding!");
    }
  }
  decoder_->decode(&(msg->events[0]), msg->events.size(), this);
}

void ApproxReconstruction::publishFrame()
{
  simpleReconstructor_.getImage(
    &imageMsgTemplate_.data[0], imageMsgTemplate_.step);
  auto msg = std::make_unique<sensor_msgs::Image>(imageMsgTemplate_);
  ros::Time t;
  t.fromNSec(nextFrameTime_);
  msg->header.stamp = t;
  imagePub_.publish(std::move(msg));
}

}  // namespace simple_image_recon
