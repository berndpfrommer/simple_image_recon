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

namespace simple_image_recon
{
ApproxReconstruction::ApproxReconstruction(ros::NodeHandle & nh) : nh_(nh)
{
  const double fps = nh_.param<double>("fps", 25.0);
  const int cutoffNumEvents = nh_.param<int>("cutoff_num_events", 30);
  const double fillRatio = nh_.param<double>("fill_ratio", 0.6);
  const int tileSize = nh_.param<int>("tile_size", 2);
  const double offset = nh_.param<double>("time_offset", 0);
  reconstructor_ = std::make_unique<ApproxRecon>(
    this, std::string("unused"), cutoffNumEvents, fps, fillRatio, tileSize,
    static_cast<uint64_t>(std::abs(offset * 1e9)));

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

}  // namespace simple_image_recon
