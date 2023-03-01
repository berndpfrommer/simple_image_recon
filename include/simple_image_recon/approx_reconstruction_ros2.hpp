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

#ifndef SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_
#define SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_

#include <event_array_codecs/decoder_factory.h>

#include <event_array_msgs/msg/event_array.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

#include "simple_image_recon/approx_reconstructor_ros2.hpp"
#include "simple_image_recon/frame_handler_ros2.hpp"

namespace simple_image_recon
{
class ApproxReconstruction : public rclcpp::Node,
                             public simple_image_recon::FrameHandler
{
public:
  using EventArray = event_array_msgs::msg::EventArray;
  explicit ApproxReconstruction(const rclcpp::NodeOptions & options);
  ~ApproxReconstruction();

  void frame(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const std::string & topic) override;

private:
  void subscriptionCheckTimerExpired();
  void eventMsg(EventArray::ConstSharedPtr msg);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  rclcpp::Subscription<event_array_msgs::msg::EventArray>::SharedPtr eventSub_;
  image_transport::Publisher imagePub_;
  std::unique_ptr<ApproxReconstructor> reconstructor_;
};
}  // namespace simple_image_recon
#endif  // SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_
