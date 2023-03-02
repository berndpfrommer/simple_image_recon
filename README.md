# simple_image_recon

ROS/ROS2 package to perform simple image reconstruction for an event
based camera. The node takes as input
[event_array_msgs](https://github.com/berndpfrommer/event_array_msgs). The
core logic of event reconstruction can be found in the
[simple image recon library package](https://github.com/berndpfrommer/simple_image_recon_lib).

## Supported platforms

Currently tested on Ubuntu 20.04 under ROS Noetic and ROS2
Galactic. Continuous integration testing also for Ubuntu 22.04 under
ROS2 Humble.

## How to build
Create a workspace (``~/ws``), clone this repo, and use ``vcs`` tool
```
pkg=simple_image_recon
mkdir -p ~/${pkg}/src
cd ~/${pkg}/src
git clone https://github.com/berndpfrommer/${pkg}.git
vcs import < ${pkg}/${pkg}.repos
cd ..
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## How to use

Launch the
[event based camera driver](https://github.com/berndpfrommer/metavision_ros_driver) and
start the reconstruction node:

ROS1:
```
roslaunch simple_image_recon node.launch camera:=event_camera
rqt_image_view
```

ROS2:
```
ros2 launch simple_image_recon node.launch.py camera:=event_camera
ros2 run rqt_image_view rqt_image_view
```
Parameters:

- ``fps`` Frequency (in hz) at which images are emitted. Default: 25.
- ``fill_ratio`` Required fill ratio for event filtering. Leave at 0.5.
- ``tile_size`` Tile size for event filtering. Leave at 2.
- ``cutoff_num_events``  Cutoff period for the temporal filter, default: 7.


## Convert bag to images

You can convert events from a bag directly into images. Run
``bag_to_frames`` with ``-h`` for documentation. It can also convert
synchronized events from multiple event cameras into synchronized frames.

ROS1:
```
rosrun simple_image_recon bag_to_frames -i input_bag -o output_bag -t /event_cam_0/events -t /event_cam_1/events -f 5.0 -O 3.028
```
The ``-t`` option can be repeated to specify the topics in a multi-camera environment. The ``-O`` option permits a time offset (in seconds). It can be repeated to apply to the first, second etc camera, but must be positive. In this case the time of camera 0 is shifted by 3.028 seconds to bring the event streams into sync.

ROS2:
```
ros2 run simple_image_recon bag_to_frames -i input_bag -o output_bag -t /event_cam_0/events -t /event_cam_1/events -f 5.0
```

## License

This software is issued under the Apache License Version 2.0.
