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

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <memory>

#include "simple_image_recon/approx_reconstruction_ros1.hpp"

namespace simple_image_recon
{
class ApproxReconstructionNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    node_ = std::make_shared<ApproxReconstruction>(nh_);
  }

private:
  // ------ variables --------
  std::shared_ptr<ApproxReconstruction> node_;
  ros::NodeHandle nh_;
};
}  // namespace simple_image_recon

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  simple_image_recon::ApproxReconstructionNodelet, nodelet::Nodelet)
