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
from launch_ros.actions import Node  # noqa: E402
from ament_index_python.packages import get_package_share_directory


USB_CAM_DIR = get_package_share_directory('usb_cam')

def generate_launch_description():
    ld = LaunchDescription()

    logitech_09 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_09',
            namespace='logitech_09',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_09'}],
            remappings = [
                ('image_raw', 'logitech_09/image_raw'),
                ('image_raw/compressed', 'logitech_09/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_09/compressedDepth'),
                ('image_raw/theora', 'logitech_09/image_raw/theora'),
                ('camera_info', 'logitech_09/camera_info'),
            ]
        )
    
    logitech_08 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_08',
            namespace='logitech_08',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_08'}],
            remappings = [
                ('image_raw', 'logitech_08/image_raw'),
                ('image_raw/compressed', 'logitech_08/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_08/compressedDepth'),
                ('image_raw/theora', 'logitech_08/image_raw/theora'),
                ('camera_info', 'logitech_08/camera_info'),
            ]
        )
    
    logitech_07 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_07',
            namespace='logitech_07',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_07'},
                        {'camera_parameters_url' : str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_07/image_raw'),
                ('image_raw/compressed', 'logitech_07/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_07/compressedDepth'),
                ('image_raw/theora', 'logitech_07/image_raw/theora'),
                ('camera_info', 'logitech_07/camera_info'),
            ]
        )

    camera_nodes = [
        logitech_07,
        # logitech_08,
        # logitech_09
    ]

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    return ld
