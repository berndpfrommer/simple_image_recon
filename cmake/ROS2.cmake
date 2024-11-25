# -*- cmake -*-
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# the rosbag api changed between distros
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR
      $ENV{ROS_DISTRO} STREQUAL "galactic")
    add_definitions(-DUSE_OLD_ROSBAG_API)
  endif()
else()
  message(ERROR "ROS_DISTRO environment variable is not set!")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(simple_image_recon_lib REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

#
# --------- library
#
add_library(approx_reconstruction
  src/approx_reconstruction_ros2.cpp)
target_include_directories(
    approx_reconstruction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(approx_reconstruction
  rclcpp rclcpp_components
  event_camera_codecs event_camera_msgs sensor_msgs
  image_transport)
# need to link this target separately or else it won't find the header files
target_link_libraries(approx_reconstruction simple_image_recon_lib::simple_image_recon_lib)

rclcpp_components_register_nodes(approx_reconstruction "simple_image_recon::ApproxReconstruction")

#
# -------- node
#
add_executable(approx_reconstruction_node src/node_ros2.cpp)
target_link_libraries(approx_reconstruction_node approx_reconstruction)

#
# -------- bag_to_frames
#
add_executable(bag_to_frames src/bag_to_frames_ros2.cpp)
target_include_directories(bag_to_frames PUBLIC include)
ament_target_dependencies(bag_to_frames
  rclcpp rclcpp_components
  event_camera_codecs event_camera_msgs sensor_msgs cv_bridge
  rosbag2_cpp)

target_link_libraries(bag_to_frames
  simple_image_recon_lib::simple_image_recon_lib
  opencv_core opencv_imgcodecs)

# the nodes must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  approx_reconstruction_node
  bag_to_frames
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  approx_reconstruction
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

ament_package()
