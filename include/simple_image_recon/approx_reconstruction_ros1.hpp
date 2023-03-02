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

#ifndef SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS1_HPP_
#define SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS1_HPP_

#include <event_array_codecs/decoder_factory.h>
#include <event_array_codecs/event_processor.h>
#include <event_array_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

#include "simple_image_recon/approx_reconstructor.hpp"
#include "simple_image_recon/frame_handler.hpp"

namespace simple_image_recon
{
class ApproxReconstruction
: public simple_image_recon::FrameHandler<sensor_msgs::Image::ConstPtr>
{
public:
  using EventArray = event_array_msgs::EventArray;
  explicit ApproxReconstruction(ros::NodeHandle & nh);
  ~ApproxReconstruction();

  void frame(
    const sensor_msgs::Image::ConstPtr & img, const std::string &) override
  {
    imagePub_.publish(std::move(img));
  }

private:
  void publishFrame();
  void eventMsg(const EventArray::ConstPtr & msg)
  {
    reconstructor_->processMsg(msg);
  }
  void imageConnectCallback(const image_transport::SingleSubscriberPublisher &);

  // ------------------------  variables ------------------------------
  using ApproxRecon = ApproxReconstructor<
    EventArray, EventArray::ConstPtr, sensor_msgs::Image,
    sensor_msgs::Image::ConstPtr>;
  ros::NodeHandle nh_;
  ros::Subscriber eventSub_;
  bool isSubscribedToEvents_{false};
  image_transport::Publisher imagePub_;
  std::unique_ptr<ApproxRecon> reconstructor_;
};
}  // namespace simple_image_recon
#endif  // SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS1_HPP_
