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

#ifndef SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTOR_HPP_
#define SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTOR_HPP_

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

#include <chrono>
#include <memory>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

#include "simple_image_recon/check_endian.hpp"
#include "simple_image_recon/frame_handler.hpp"

namespace simple_image_recon
{
template <
  typename EventPacketT, typename EventPacketConstSharedPtrT, typename ImageT,
  typename ImageConstPtrT>
class ApproxReconstructor : public event_camera_codecs::EventProcessor
{
public:
  using EventPacket = EventPacketT;
  explicit ApproxReconstructor(
    FrameHandler<ImageConstPtrT> * fh, const std::string & topic,
    int cutoffNumEvents = 30, double fps = 25.0, double fillRatio = 0.6,
    int tileSize = 2, uint64_t offset = 0, uint64_t rosOffset = 0)
  : frameHandler_(fh),
    topic_(topic),
    cutoffNumEvents_(cutoffNumEvents),
    fillRatio_(fillRatio),
    tileSize_(tileSize),
    timeOffset_(offset),
    rosOffset_(rosOffset)
  {
    sliceInterval_ = static_cast<uint64_t>(1000000000 / std::abs(fps));
    imageMsgTemplate_.height = 0;
  }

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    simpleReconstructor_.event(t, ex, ey, polarity);
    while (t + timeOffset_ > nextFrameTime_) {
      emitFrame();
      nextFrameTime_ += sliceInterval_;
    }
    numEvents_++;
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // --------- end of inherited from EventProcessor

  uint64_t getT0() const { return (t0_); }
  const auto & getNumEvents() const { return (numEvents_); }
  const auto & getTimeElapsed() const { return (timeElapsed_); }
  const auto & getTotalWindowSize() const { return (totalQueueSize_); }
  const auto & getNumMessages() const { return (numMessages_); }

  void processMsg(EventPacketConstSharedPtrT msg)
  {
    if (imageMsgTemplate_.height == 0) {
      imageMsgTemplate_.header = msg->header;
      imageMsgTemplate_.width = msg->width;
      imageMsgTemplate_.height = msg->height;
      imageMsgTemplate_.encoding = "mono8";
      imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
      imageMsgTemplate_.step = imageMsgTemplate_.width;
      // imageMsgTemplate_.data.resize(msg->width * msg->height, 0);
      FirstMsgProcessor firstMsgProcessor;
      event_camera_codecs::DecoderFactory<EventPacket, FirstMsgProcessor>
        firstFactory;
      auto firstDecoder = firstFactory.newInstance(*msg);
      firstDecoder->decode(*msg, &firstMsgProcessor);
      t0_ = firstMsgProcessor.getFirstTimeStamp() + timeOffset_;
      nextFrameTime_ = (t0_ / sliceInterval_) * sliceInterval_;
      simpleReconstructor_.initialize(
        msg->width, msg->height,
        static_cast<uint32_t>(std::abs(cutoffNumEvents_)), tileSize_,
        fillRatio_);
      decoder_ = decoderFactory_.newInstance(*msg);
      if (!decoder_) {
        std::cerr << "invalid encoding: " << msg->encoding << std::endl;
        throw(std::runtime_error("invalid encoding!"));
      }
    }
    const auto start = std::chrono::high_resolution_clock::now();
    decoder_->decode(*msg, this);
    const decltype(start) final = std::chrono::high_resolution_clock::now();
    timeElapsed_ +=
      std::chrono::duration_cast<std::chrono::microseconds>(final - start)
        .count();
    totalQueueSize_ += simpleReconstructor_.getQueueSize();
    numMessages_++;
  }

private:
  // special handling for first message
  class FirstMsgProcessor : public event_camera_codecs::EventProcessor
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
    void finished() override {}
    void rawData(const char *, size_t) override {}
    // --------- end of inherited from EventProcessor
    uint64_t getFirstTimeStamp() const { return (firstTimeStamp_); }
    // --------- variables -----------
  private:
    uint64_t firstTimeStamp_{0};
  };

  void emitFrame()
  {
    auto msg = std::make_unique<ImageT>(imageMsgTemplate_);
    msg->data.resize(msg->height * msg->step);
    simpleReconstructor_.getImage(&(msg->data[0]), msg->step);
#ifdef USING_ROS_1
    ros::Time t;
    t.fromNSec(nextFrameTime_ + rosOffset_);
    msg->header.stamp = t;
#else
    msg->header.stamp =
      rclcpp::Time(nextFrameTime_ + rosOffset_, RCL_SYSTEM_TIME);
#endif
    frameHandler_->frame(std::move(msg), topic_);
  }

  // ------------------------  variables ------------------------------
  FrameHandler<ImageConstPtrT> * frameHandler_{nullptr};
  std::string topic_;
  ImageT imageMsgTemplate_;
  int cutoffNumEvents_{0};
  uint64_t sliceInterval_{0};
  uint64_t nextFrameTime_{0};
  uint64_t t0_{0};
  double fillRatio_{0};
  int tileSize_{0};
  uint64_t timeOffset_{0};
  uint64_t rosOffset_{0};
  uint64_t numEvents_{0};
  uint64_t timeElapsed_{0};  // in usec
  uint64_t totalQueueSize_{0};
  uint64_t numMessages_{0};
  std::shared_ptr<
    event_camera_codecs::Decoder<EventPacketT, ApproxReconstructor>>
    decoder_;
  event_camera_codecs::DecoderFactory<EventPacketT, ApproxReconstructor>
    decoderFactory_;
  simple_image_recon_lib::SimpleImageReconstructor simpleReconstructor_;
};
}  // namespace simple_image_recon
#endif  // SIMPLE_IMAGE_RECON__APPROX_RECONSTRUCTOR_HPP_
