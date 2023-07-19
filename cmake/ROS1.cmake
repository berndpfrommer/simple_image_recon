# -*- cmake -*-
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
add_definitions(-DUSING_ROS_1)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(simple_image_recon_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  rosbag
  event_camera_msgs
  event_camera_codecs
  sensor_msgs
  image_transport)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS})
#
# --------- library
#
add_library(approx_reconstruction
  src/approx_reconstruction_ros1.cpp)

target_include_directories(
    approx_reconstruction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

set(ALL_LIBS simple_image_recon_lib::simple_image_recon_lib ${catkin_LIBRARIES})

# need to link this target separately or else it won't find the header files
target_link_libraries(approx_reconstruction ${ALL_LIBS})

#
# --------- nodelet
#
add_library(approx_reconstruction_nodelet src/nodelet.cpp)
target_link_libraries(approx_reconstruction_nodelet approx_reconstruction ${ALL_LIBS})

#
# -------- node
#
add_executable(approx_reconstruction_node src/node_ros1.cpp)
target_link_libraries(approx_reconstruction_node approx_reconstruction ${ALL_LIBS})

#
# -------- bag_to_frames
#

add_executable(bag_to_frames src/bag_to_frames_ros1.cpp)
target_include_directories(bag_to_frames PUBLIC include)
target_link_libraries(bag_to_frames approx_reconstruction ${ALL_LIBS})


# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  approx_reconstruction_node
  bag_to_frames
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  approx_reconstruction
  DESTINATION lib
)
install(TARGETS approx_reconstruction approx_reconstruction_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch")
