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

#include "simple_image_recon/approx_reconstruction_ros2.hpp"

#include <event_array_msgs/msg/event_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "simple_image_recon/check_endian.hpp"

namespace simple_image_recon
{
ApproxReconstruction::ApproxReconstruction(const rclcpp::NodeOptions & options)
: Node(
    "event_approx_reconstruction",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
{
  double fps;
  this->get_parameter_or("fps", fps, 25.0);
  int cutoffNumEvents{30};
  this->get_parameter_or("cutoff_num_events", cutoffNumEvents, 30);

  RCLCPP_INFO_STREAM(get_logger(), "cutoff number events: " << cutoffNumEvents);
  double fillRatio{0.6};
  this->get_parameter_or("fill_ratio", fillRatio, 0.6);
  RCLCPP_INFO_STREAM(get_logger(), "fill ratio: " << fillRatio);
  int tileSize{2};
  this->get_parameter_or("tile_size", tileSize, 2);
  RCLCPP_INFO_STREAM(get_logger(), "tile size: " << tileSize);

  reconstructor_ = std::make_unique<ApproxReconstructor>(
    this, std::string("unused"), cutoffNumEvents, fps, fillRatio, tileSize);

  const rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  imagePub_ = image_transport::create_publisher(this, "~/image_raw", qosProf);
  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&ApproxReconstruction::subscriptionCheckTimerExpired, this));
}

ApproxReconstruction::~ApproxReconstruction()
{
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
  }
}

void ApproxReconstruction::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if (imagePub_.getNumSubscribers()) {
    // -------------- subscribers ---------------------
    if (!eventSub_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to events!");
      const int qsize = 1000;
      auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize))
                   .best_effort()
                   .durability_volatile();
      eventSub_ = this->create_subscription<event_array_msgs::msg::EventArray>(
        "~/events", qos,
        std::bind(
          &ApproxReconstruction::eventMsg, this, std::placeholders::_1));
    }
  } else {
    // -------------- no subscribers -------------------
    if (eventSub_) {
      RCLCPP_INFO(this->get_logger(), "unsubscribing from events!");
      eventSub_.reset();
    }
  }
}

void ApproxReconstruction::eventMsg(EventArray::ConstSharedPtr msg)
{
  reconstructor_->processMsg(msg);
}

void ApproxReconstruction::frame(const sensor_msgs::msg::Image::ConstSharedPtr & img,
                                 const std::string &)
{
  imagePub_.publish(std::move(img));
}

}  // namespace simple_image_recon

RCLCPP_COMPONENTS_REGISTER_NODE(simple_image_recon::ApproxReconstruction)
