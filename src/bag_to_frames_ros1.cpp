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

#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <unistd.h>

#include <memory>

#include "simple_image_recon/approx_reconstructor.hpp"
#include "simple_image_recon/frame_handler.hpp"

using event_camera_msgs::EventPacket;
using sensor_msgs::Image;
using ApproxRecon = simple_image_recon::ApproxReconstructor<
  EventPacket, EventPacket::ConstPtr, Image, Image::ConstPtr>;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -i input_bag -o output_bag "
               "[-O time_offset_sec] [-t input_topic] "
               "[-T output_topic] [-f fps] [-c cutoff_period]"
            << std::endl;
}

class OutBagWriter : public simple_image_recon::FrameHandler<Image::ConstPtr>
{
public:
  explicit OutBagWriter(const std::string & bagName)
  {
    outBag_.open(bagName, rosbag::bagmode::Write);
  }
  void frame(const Image::ConstPtr & img, const std::string & topic) override
  {
    outBag_.write(topic, img->header.stamp, img);
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

private:
  rosbag::Bag outBag_;
  size_t numFrames_{0};
};

using simple_image_recon::ApproxReconstructor;

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outBagName;
  std::vector<std::string> inTopics;
  std::vector<std::string> outTopics;
  int cutoff_period(30);
  std::vector<uint64_t> offsets;
  double fps(25);
  while ((opt = getopt(argc, argv, "i:o:O:t:T:f:c:h")) != -1) {
    switch (opt) {
      case 'i':
        inBagName = optarg;
        break;
      case 'o':
        outBagName = optarg;
        break;
      case 'O':
        offsets.push_back(static_cast<uint64_t>(std::abs(atof(optarg)) * 1e9));
        break;
      case 't':
        inTopics.push_back(std::string(optarg));
        break;
      case 'T':
        outTopics.push_back(std::string(optarg));
        break;
      case 'f':
        fps = atof(optarg);
        break;
      case 'c':
        cutoff_period = atoi(optarg);
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }

  if (inBagName.empty()) {
    std::cout << "missing input bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (outBagName.empty()) {
    std::cout << "missing output bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (inTopics.empty()) {
    std::cout << "no input topics found!" << std::endl;
    return (-1);
  }
  if (outTopics.empty()) {
    for (const auto s : inTopics) {
      outTopics.push_back(s + "/image_raw");
    }
  }

  while (offsets.size() < inTopics.size()) {
    offsets.push_back(0);
  }

  if (outTopics.size() != inTopics.size()) {
    std::cout << "must have same number of input and output topics!"
              << std::endl;
    return (-1);
  }

  if (fps < 1e-5) {
    std::cout << "fps too small: " << fps << std::endl;
    return (-1);
  }

  const double fillRatio = 0.6;
  const int tileSize = 2;
  OutBagWriter writer(outBagName);
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < inTopics.size(); i++) {
    recons.insert(
      {inTopics[i], ApproxRecon(
                      &writer, outTopics[i], cutoff_period, fps, fillRatio,
                      tileSize, offsets[i])});
  }

  rosbag::Bag inBag;
  inBag.open(inBagName, rosbag::bagmode::Read);

  size_t numMessages(0);
  rosbag::View view(inBag, rosbag::TopicQuery(inTopics));
  for (const rosbag::MessageInstance & msg : view) {
    auto it = recons.find(msg.getTopic());
    if (it != recons.end()) {
      auto m = msg.instantiate<EventPacket>();
      it->second.processMsg(m);
      numMessages++;
    }
  }
  std::cout << "processed " << numMessages << " number of messages"
            << std::endl;

  return (0);
}
