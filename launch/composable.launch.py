# -----------------------------------------------------------------------------
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
#
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    container = ComposableNodeContainer(
            name='simple_image_recon_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='simple_image_recon',
                    plugin='simple_image_recon::ApproxReconstruction',
                    namespace=LaunchConfig('camera'),
                    parameters=[
                        {'fps': LaunchConfig('fps'),
                         'cutoff_num_events': LaunchConfig('cutoff_num_events'),
                         'fill_ratio': 0.5,
                         'tile_size': 2}],
                    remappings=[('~/events', LaunchConfig('topic'))])
            ],
            output='screen',
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('camera', default_value=['event_camera'],
                  description='camera'),
        LaunchArg('fps', default_value='25.0',
                  description='frame rate'),
        LaunchArg('cutoff_num_events', default_value='7',
                  description='number of events for temporal filter'),
        LaunchArg('topic', default_value='events',
                  description='topic for events'),
        OpaqueFunction(function=launch_setup)
        ])
