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

#include <unistd.h>

#include <algorithm>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <simple_image_recon/approx_reconstructor.hpp>
#include <simple_image_recon/frame_handler.hpp>
#include <vector>

using event_camera_msgs::msg::EventPacket;
using sensor_msgs::msg::Image;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -i input_bag -o output_bag "
               "[-O time_offset_sec] [-r ros_pulse_time_nsec] "
               "[-p pulse_time_nsec] [-t input_topic] "
               "[-T output_topic] [-f fps] [-c cutoff_period]"
            << std::endl;
}

class OutBagWriter
: public simple_image_recon::FrameHandler<Image::ConstSharedPtr>
{
public:
  explicit OutBagWriter(
    const std::string & bagName, const std::vector<std::string> & outTopics)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bagName);
    for (const auto & topic : outTopics) {
      struct rosbag2_storage::TopicMetadata md;
      md.name = topic;
      md.type = "sensor_msgs/msg/Image";
      md.serialization_format = rmw_get_serialization_format();
      md.offered_qos_profiles = "";
      writer_->create_topic(md);
    }
  }

  void frame(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const std::string & topic) override
  {
    rclcpp::Serialization<Image> serialization;
#ifdef USE_OLD_ROSBAG_API
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(img.get(), &serialized_msg);
#else
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(img.get(), serialized_msg.get());
#endif
    writer_->write(
      serialized_msg, topic, "sensor_msgs/msg/Image",
      rclcpp::Time(img->header.stamp));
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  size_t numFrames_{0};
};

using ApproxRecon = simple_image_recon::ApproxReconstructor<
  EventPacket, EventPacket::ConstSharedPtr, Image, Image::ConstSharedPtr>;

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outBagName;
  std::vector<std::string> inTopics;
  std::vector<std::string> outTopics;
  int cutoff_period(30);
  std::vector<uint64_t> offsets;
  std::vector<uint64_t> pulses;
  std::vector<uint64_t> rosPulses;
  double fps(25);
  while ((opt = getopt(argc, argv, "i:o:O:t:T:p:r:f:c:h")) != -1) {
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
      case 'p':
        pulses.push_back(static_cast<uint64_t>(atoll(optarg)));
        break;
      case 'r':
        rosPulses.push_back(static_cast<uint64_t>(atoll(optarg)));
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
        std::cout << "got unknown option!" << std::endl;
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
    for (const auto & s : inTopics) {
      outTopics.push_back(s + "/image_raw");
    }
  }

  if (!(offsets.empty() || pulses.empty())) {
    std::cout << "can only specify offset or pulses!" << std::endl;
    return (-1);
  }

  std::vector<uint64_t> rosOffsets;
  if (offsets.empty()) {
    if (rosPulses.size() != pulses.size()) {
      std::cout << "missing ROS pulse times (option -r)!" << std::endl;
      return (-1);
    }
    const auto ix = std::distance(
      std::begin(pulses),
      std::max_element(std::begin(pulses), std::end(pulses)));

    for (size_t i = 0; i < pulses.size(); ++i) {
      const auto & p = pulses[i];
      offsets.push_back(pulses[ix] - p);  // will be added to sensor time
      rosOffsets.push_back(rosPulses[ix]);
    }
  }

  while (offsets.size() < inTopics.size()) {
    offsets.push_back(0);
  }
  while (rosOffsets.size() < inTopics.size()) {
    rosOffsets.push_back(0);
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
  OutBagWriter writer(outBagName, outTopics);
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < inTopics.size(); i++) {
    recons.insert(
      {inTopics[i], ApproxRecon(
                      &writer, outTopics[i], cutoff_period, fps, fillRatio,
                      tileSize, offsets[i], rosOffsets[i])});
  }

  rosbag2_cpp::Reader reader;
  reader.open(inBagName);
  rclcpp::Serialization<EventPacket> serialization;
  size_t numMessages(0);
  while (reader.has_next()) {
    auto msg = reader.read_next();
    auto it = recons.find(msg->topic_name);
    if (it != recons.end()) {
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      std::shared_ptr<EventPacket> m(new EventPacket());
      serialization.deserialize_message(&serializedMsg, m.get());
      it->second.processMsg(m);
      numMessages++;
    }
  }
  for (const auto & me : recons) {
    std::cout << "first time (adjusted) " << me.first << " "
              << me.second.getT0() << std::endl;
  }
  std::cout << "processed " << numMessages << " number of messages"
            << std::endl;

  return (0);
}
