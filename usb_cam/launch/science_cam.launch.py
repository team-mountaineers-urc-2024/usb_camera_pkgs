# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node, RosTimer  # noqa: E402
from ament_index_python.packages import get_package_share_directory


USB_CAM_DIR = get_package_share_directory('usb_cam')

def generate_launch_description():
    ld = LaunchDescription()

    logitech_14 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_14',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_14'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_14/image_raw'),
                ('image_raw/compressed', 'logitech_14/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_14/compressedDepth'),
                ('image_raw/theora', 'logitech_14/image_raw/theora'),
                ('camera_info', 'logitech_14/camera_info'),
            ]
        )
    
    logitech_14_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_14_mux',
            # namespace='logitech_14',
            parameters= [
                {'theora_input' : 'logitech_14/image_raw/theora'},
                {'theora_output' : 'logitech_14/image_raw/theora_mux'},
                {'theora_service' : 'logitech_14_mux/recall_header'}],
        )
    
    logitech_15 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_15',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_15'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_15/image_raw'),
                ('image_raw/compressed', 'logitech_15/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_15/compressedDepth'),
                ('image_raw/theora', 'logitech_15/image_raw/theora'),
                ('camera_info', 'logitech_15/camera_info'),
            ]
        )
    
    logitech_15_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_15_mux',
            # namespace='logitech_15',
            parameters= [
                {'theora_input' : 'logitech_15/image_raw/theora'},
                {'theora_output' : 'logitech_15/image_raw/theora_mux'},
                {'theora_service' : 'logitech_15_mux/recall_header'}],
        )

    logitech_16 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_16',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_16'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_16/image_raw'),
                ('image_raw/compressed', 'logitech_16/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_16/compressedDepth'),
                ('image_raw/theora', 'logitech_16/image_raw/theora'),
                ('camera_info', 'logitech_16/camera_info'),
            ]
        )
    
    logitech_16_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_16_mux',
            # namespace='logitech_16',
            parameters= [
                {'theora_input' : 'logitech_16/image_raw/theora'},
                {'theora_output' : 'logitech_16/image_raw/theora_mux'},
                {'theora_service' : 'logitech_16_mux/recall_header'}],
        )
    
    logitech_17 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_17',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_17'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_17/image_raw'),
                ('image_raw/compressed', 'logitech_17/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_17/compressedDepth'),
                ('image_raw/theora', 'logitech_17/image_raw/theora'),
                ('camera_info', 'logitech_17/camera_info'),
            ]
        )
    
    logitech_17_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_17_mux',
            # namespace='logitech_17',
            parameters= [
                {'theora_input' : 'logitech_17/image_raw/theora'},
                {'theora_output' : 'logitech_17/image_raw/theora_mux'},
                {'theora_service' : 'logitech_17_mux/recall_header'}],
        )
    
    logitech_16_d = RosTimer(period = 0.0, actions = [logitech_16])
    logitech_15_d = RosTimer(period = 2.0, actions = [logitech_15])
    logitech_14_d = RosTimer(period = 4.0, actions = [logitech_14])
    logitech_17_d = RosTimer(period = 6.0, actions = [logitech_17])

    camera_nodes = [
        logitech_16_d,
        logitech_15_d,
        logitech_14_d,
        logitech_17_d,
    ]

    mux_nodes = [
        logitech_14_mux,
        logitech_15_mux,
        logitech_16_mux,
        logitech_17_mux,
    ]

    camera_group = GroupAction(camera_nodes)
    mux_group = GroupAction(mux_nodes)

    ld.add_action(camera_group)
    ld.add_action(mux_group)
    return ld
