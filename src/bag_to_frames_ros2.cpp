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
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
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
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

using event_camera_msgs::msg::EventPacket;
using sensor_msgs::msg::Image;
using FrameHandler = simple_image_recon::FrameHandler<Image::ConstSharedPtr>;
void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -i input_bag -o output_dir/bag -t input_topic "
               "[-O time_offset_sec] [-r ros_pulse_time_nsec] "
               "[-p pulse_time_nsec] "
               "[-b (write to bag)] "
               "[-s tile_size] "
               "[-T output_topic] [-f fps] [-c cutoff_period]"
            << std::endl;
}

class OutBagWriter : public FrameHandler
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
      // md.offered_qos_profiles = "";
      writer_->create_topic(md);
    }
  }
  virtual ~OutBagWriter(){};
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

class FrameWriter : public FrameHandler
{
public:
  struct FileInfo
  {
    FileInfo(const std::string & dir_name) : dir(dir_name) {}
    std::string dir;
    size_t counter{0};
  };

  explicit FrameWriter(
    const std::string & base_name, const std::vector<std::string> & outTopics)
  {
    for (const auto & topic : outTopics) {
      const std::string dir = base_name + topic;
      dir_map_.insert({topic, FileInfo(dir)});
      std::filesystem::create_directories(dir);  // does recursive...
    }
  }

  void frame(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const std::string & topic) override
  {
    auto & file_info = dir_map_.find(topic)->second;
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << file_info.counter;
    const auto fname = file_info.dir + "/frame_" + ss.str() + ".jpg";
    auto cvImg = cv_bridge::toCvShare(img, "mono8");
    cv::imwrite(fname, cvImg->image);
    file_info.counter++;
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

private:
  std::map<std::string, FileInfo> dir_map_;
  size_t numFrames_{0};
};

struct ApproxRecon
{
  ApproxRecon(
    simple_image_recon::FrameHandler<Image::ConstSharedPtr> * fh,
    const std::string & topic, int cutoff_period = 30, double fps = 25.0,
    double fillRatio = 0.6, int tileSize = 2, uint64_t offset = 0,
    uint64_t rosOffset = 0)
  : recon(fh, topic, cutoff_period, fps, fillRatio, tileSize, offset, rosOffset)
  {
  }
  using Recon = simple_image_recon::ApproxReconstructor<
    EventPacket, EventPacket::ConstSharedPtr, Image, Image::ConstSharedPtr>;
  Recon recon;
  uint32_t width{0};
  uint32_t height{0};
};

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outName;
  std::vector<std::string> inTopics;
  std::vector<std::string> outTopics;
  int cutoff_period(30);
  std::vector<uint64_t> offsets;
  std::vector<uint64_t> pulses;
  std::vector<uint64_t> rosPulses;
  double fps(25);
  int tileSize = 2;
  bool writeBag(false);
  while ((opt = getopt(argc, argv, "i:o:O:t:T:p:r:s:f:c:hb")) != -1) {
    switch (opt) {
      case 'i':
        inBagName = optarg;
        break;
      case 'o':
        outName = optarg;
        break;
      case 'b':
        writeBag = true;
        break;
      case 's':
        tileSize = atoi(optarg);
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

  if (outName.empty()) {
    std::cout << "missing output bag / dir file name!" << std::endl;
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
  std::unordered_map<std::string, ApproxRecon> recons;
  std::unique_ptr<FrameHandler> fh(
    writeBag
      ? dynamic_cast<FrameHandler *>(new OutBagWriter(outName, outTopics))
      : dynamic_cast<FrameHandler *>(new FrameWriter(outName, outTopics)));
  for (size_t i = 0; i < inTopics.size(); i++) {
    recons.insert(
      {inTopics[i], ApproxRecon(
                      fh.get(), outTopics[i], cutoff_period, fps, fillRatio,
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
      if (numMessages == 0) {
        it->second.width = m->width;
        it->second.height = m->height;
      }
      it->second.recon.processMsg(m);
      numMessages++;
    }
#if 0    
    std::cout << it->second.recon.getTotalWindowSize() /
                   static_cast<double>(it->second.recon.getNumMessages())
              << std::endl;
#endif
  }
  for (const auto & me : recons) {
    const auto & recon = me.second.recon;
    const auto n = recon.getNumEvents();
    const auto usecs = recon.getTimeElapsed();
    const auto n_msgs = static_cast<double>(recon.getNumMessages());
    std::cout << "topic:  " << me.first << std::endl;
    std::cout << "resolution: " << me.second.width << " x " << me.second.height
              << std::endl;
    std::cout << "events: " << n << std::endl;
    const double r = (usecs != 0) ? n / static_cast<double>(usecs) : -1.0;
    std::cout << "reconstruction rate: " << r << " Mevs" << std::endl;
    std::cout << "time per event: " << usecs * 1000.0 / static_cast<double>(n)
              << "ns" << std::endl;
    std::cout << "first time (adjusted): " << recon.getT0() << std::endl;

    std::cout << "average window size: "
              << recon.getTotalWindowSize() / (n_msgs != 0 ? n_msgs : 1.0)
              << std::endl;
    ;
  }
  std::cout << "processed " << numMessages << " number of messages"
            << std::endl;

  return (0);
}
