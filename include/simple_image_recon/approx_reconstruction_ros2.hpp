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

#ifndef SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_
#define SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_

#include <event_array_codecs/decoder_factory.h>
#include <event_array_codecs/event_processor.h>

#include <event_array_msgs/msg/event_array.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

namespace simple_image_recon
{
class ApproxReconstruction : public rclcpp::Node,
                             event_array_codecs::EventProcessor
{
public:
  using EventArray = event_array_msgs::msg::EventArray;
  explicit ApproxReconstruction(const rclcpp::NodeOptions & options);
  ~ApproxReconstruction();

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    simpleReconstructor_.event(ex, ey, polarity);
    while (t > nextFrameTime_) {
      publishFrame();
      nextFrameTime_ += sliceInterval_;
    }
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --------- end of inherited from EventProcessor

private:
  // special handling for first message
  class FirstMsgProcessor : public event_array_codecs::EventProcessor
  {
    // ---------- inherited from EventProcessor
  public:
    void eventCD(uint64_t t, uint16_t, uint16_t, uint8_t) override
    {
      if (firstTimeStamp_ == 0) {
        firstTimeStamp_ = t;
      }
    }
    void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
    void finished() override{};
    void rawData(const char *, size_t) override{};
    // --------- end of inherited from EventProcessor
    uint64_t getFirstTimeStamp() const { return (firstTimeStamp_); }
    // --------- variables -----------
  private:
    uint64_t firstTimeStamp_{0};
  };

  void subscriptionCheckTimerExpired();
  void publishFrame();
  void eventMsg(EventArray::ConstSharedPtr msg);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  double sliceTime_;  // duration of one frame
  rclcpp::Subscription<event_array_msgs::msg::EventArray>::SharedPtr eventSub_;
  image_transport::Publisher imagePub_;
  sensor_msgs::msg::Image imageMsgTemplate_;
  int cutoffNumEvents_{7};
  uint64_t sliceInterval_{0};
  uint64_t nextFrameTime_{0};
  int tileSize_{2};
  double fillRatio_{0.5};
  event_array_codecs::Decoder<ApproxReconstruction> * decoder_{0};
  event_array_codecs::DecoderFactory<ApproxReconstruction> decoderFactory_;
  simple_image_recon_lib::SimpleImageReconstructor simpleReconstructor_;
};
}  // namespace simple_image_recon
#endif  // SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTION_ROS2_HPP_
