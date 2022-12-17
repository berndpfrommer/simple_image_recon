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
Create a workspace (``~/ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
pkg=simple_image_recon
mkdir -p ~/$pkg/src
cd ~/ws
git clone https://github.com/berndpfrommer/${pkg}.git src/${pkg}
wstool init src src/${pkg}/${pkg}.rosinstall
# to update an existing space:
# wstool merge -t src src/${pkg}/${pkg}.rosinstall
# wstool update -t src
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
```

Parameters:

- ``fps`` Frequency (in hz) at which images are emitted. Default: 25.
- ``fill_ratio`` Required fill ratio for event filtering. Leave at 0.5.
- ``tile_size`` Tile size for event filtering. Leave at 2.
- ``cutoff_num_events``  Cutoff period for the temporal filter, default: 7.

## License

This software is issued under the Apache License Version 2.0.
