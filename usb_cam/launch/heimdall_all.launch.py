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

    logitech_01 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_01',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_01'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_01/image_raw'),
                ('image_raw/compressed', 'logitech_01/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_01/compressedDepth'),
                ('image_raw/theora', 'logitech_01/image_raw/theora'),
                ('camera_info', 'logitech_01/camera_info'),
            ]
        )
    
    logitech_01_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_01_mux',
            # namespace='logitech_01',
            parameters= [
                {'theora_input' : 'logitech_01/image_raw/theora'},
                {'theora_output' : 'logitech_01/image_raw/theora_mux'},
                {'theora_service' : 'logitech_01_mux/recall_header'}],
        )

    logitech_02 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_02',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_02'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_02/image_raw'),
                ('image_raw/compressed', 'logitech_02/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_02/compressedDepth'),
                ('image_raw/theora', 'logitech_02/image_raw/theora'),
                ('camera_info', 'logitech_02/camera_info'),
            ]
        )
    
    logitech_02_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_02_mux',
            # namespace='logitech_02',
            parameters= [
                {'theora_input' : 'logitech_02/image_raw/theora'},
                {'theora_output' : 'logitech_02/image_raw/theora_mux'},
                {'theora_service' : 'logitech_02_mux/recall_header'}],
        )
    
    logitech_03 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_03',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_03'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_03/image_raw'),
                ('image_raw/compressed', 'logitech_03/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_03/compressedDepth'),
                ('image_raw/theora', 'logitech_03/image_raw/theora'),
                ('camera_info', 'logitech_03/camera_info'),
            ]
        )
    
    logitech_03_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_03_mux',
            # namespace='logitech_03',
            parameters= [
                {'theora_input' : 'logitech_03/image_raw/theora'},
                {'theora_output' : 'logitech_03/image_raw/theora_mux'},
                {'theora_service' : 'logitech_03_mux/recall_header'}],
        )
    
    logitech_04 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_04',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_04'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_04/image_raw'),
                ('image_raw/compressed', 'logitech_04/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_04/compressedDepth'),
                ('image_raw/theora', 'logitech_04/image_raw/theora'),
                ('camera_info', 'logitech_04/camera_info'),
            ]
        )
    
    logitech_04_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_04_mux',
            # namespace='logitech_04',
            parameters= [
                {'theora_input' : 'logitech_04/image_raw/theora'},
                {'theora_output' : 'logitech_04/image_raw/theora_mux'},
                {'theora_service' : 'logitech_04_mux/recall_header'}],
        )

    logitech_05 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_05',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_05'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_05/image_raw'),
                ('image_raw/compressed', 'logitech_05/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_05/compressedDepth'),
                ('image_raw/theora', 'logitech_05/image_raw/theora'),
                ('camera_info', 'logitech_05/camera_info'),
            ]
        )
    
    logitech_05_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_05_mux',
            # namespace='logitech_05',
            parameters= [
                {'theora_input' : 'logitech_05/image_raw/theora'},
                {'theora_output' : 'logitech_05/image_raw/theora_mux'},
                {'theora_service' : 'logitech_05_mux/recall_header'}],
        )
    
    logitech_06 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_06',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_06'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_06/image_raw'),
                ('image_raw/compressed', 'logitech_06/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_06/compressedDepth'),
                ('image_raw/theora', 'logitech_06/image_raw/theora'),
                ('camera_info', 'logitech_06/camera_info'),
            ]
        )
    
    logitech_06_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_06_mux',
            # namespace='logitech_06',
            parameters= [
                {'theora_input' : 'logitech_06/image_raw/theora'},
                {'theora_output' : 'logitech_06/image_raw/theora_mux'},
                {'theora_service' : 'logitech_06_mux/recall_header'}],
        )
    
    logitech_10 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_10',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_10'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_10/image_raw'),
                ('image_raw/compressed', 'logitech_10/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_10/compressedDepth'),
                ('image_raw/theora', 'logitech_10/image_raw/theora'),
                ('camera_info', 'logitech_10/camera_info'),
            ]
        )
    
    logitech_10_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_10_mux',
            # namespace='logitech_10',
            parameters= [
                {'theora_input' : 'logitech_10/image_raw/theora'},
                {'theora_output' : 'logitech_10/image_raw/theora_mux'},
                {'theora_service' : 'logitech_10_mux/recall_header'}],
        )
    
    logitech_11 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_11',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_11'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_11/image_raw'),
                ('image_raw/compressed', 'logitech_11/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_11/compressedDepth'),
                ('image_raw/theora', 'logitech_11/image_raw/theora'),
                ('camera_info', 'logitech_11/camera_info'),
            ]
        )
    
    logitech_11_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_11_mux',
            # namespace='logitech_11',
            parameters= [
                {'theora_input' : 'logitech_11/image_raw/theora'},
                {'theora_output' : 'logitech_11/image_raw/theora_mux'},
                {'theora_service' : 'logitech_11_mux/recall_header'}],
        )
    
    logitech_12 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_12',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_12'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_12/image_raw'),
                ('image_raw/compressed', 'logitech_12/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_12/compressedDepth'),
                ('image_raw/theora', 'logitech_12/image_raw/theora'),
                ('camera_info', 'logitech_12/camera_info'),
            ]
        )
    
    logitech_12_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_12_mux',
            # namespace='logitech_12',
            parameters= [
                {'theora_input' : 'logitech_12/image_raw/theora'},
                {'theora_output' : 'logitech_12/image_raw/theora_mux'},
                {'theora_service' : 'logitech_12_mux/recall_header'}],
        )
    
    logitech_13 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_13',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_13'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_13/image_raw'),
                ('image_raw/compressed', 'logitech_13/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_13/compressedDepth'),
                ('image_raw/theora', 'logitech_13/image_raw/theora'),
                ('camera_info', 'logitech_13/camera_info'),
            ]
        )
    
    logitech_13_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='logitech_13_mux',
            # namespace='logitech_13',
            parameters= [
                {'theora_input' : 'logitech_13/image_raw/theora'},
                {'theora_output' : 'logitech_13/image_raw/theora_mux'},
                {'theora_service' : 'logitech_13_mux/recall_header'}],
        )
    
    arducam_01 = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='arducam_01',
            namespace='',
            parameters= [Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
                        {'video_device' : '/dev/urc/cam/arducam_01'},
                        {'camera_parameters_url': str(Path(USB_CAM_DIR, 'config', 'params_1.yaml'))}],
            remappings = [
                ('image_raw', 'arducam_01/image_raw'),
                ('image_raw/compressed', 'arducam_01/image_compressed'),
                ('image_raw/compressedDepth', 'arducam_01/compressedDepth'),
                ('image_raw/theora', 'arducam_01/image_raw/theora'),
                ('camera_info', 'arducam_01/camera_info'),
            ]
        )
    
    arducam_01_mux = \
        Node(
            package='theora_mux', executable='theora_mux_node', output='screen',
            name='arducam_01_mux',
            # namespace='logitech_05',
            parameters= [
                {'theora_input' : 'arducam_01/image_raw/theora'},
                {'theora_output' : 'arducam_01/image_raw/theora_mux'},
                {'theora_service' : 'arducam_01_mux/recall_header'}],
        )

    logitech_05_d = RosTimer(period = 0.0, actions = [logitech_05])
    logitech_04_d = RosTimer(period = 2.0, actions = [logitech_04])
    logitech_02_d = RosTimer(period = 4.0, actions = [logitech_02])
    logitech_12_d = RosTimer(period = 6.0, actions = [logitech_12])
    arducam_01_d = RosTimer(period = 8.0, actions = [arducam_01])
    logitech_01_d = RosTimer(period = 10.0, actions = [logitech_01])
    logitech_03_d = RosTimer(period = 12.0, actions = [logitech_03])
    logitech_06_d = RosTimer(period = 14.0, actions = [logitech_06])
    logitech_10_d = RosTimer(period = 16.0, actions = [logitech_10])
    logitech_13_d = RosTimer(period = 18.0, actions = [logitech_13])
    logitech_11_d = RosTimer(period = 20.0, actions = [logitech_11])


    camera_nodes = [
        logitech_05_d,
        logitech_04_d,
        logitech_02_d,
        logitech_12_d,
        arducam_01_d,
        logitech_01_d,
        logitech_03_d,
        logitech_06_d,
        logitech_10_d,
        logitech_13_d,
        logitech_11_d
    ]

    mux_nodes = [
        logitech_02_mux,
        logitech_04_mux,
        logitech_05_mux,
        logitech_12_mux,
        arducam_01_mux,
        logitech_01_mux,
        logitech_03_mux,
        logitech_06_mux,
        logitech_10_mux,
        logitech_13_mux,
        logitech_11_mux
    ]

    camera_group = GroupAction(camera_nodes)
    mux_group = GroupAction(mux_nodes)

    ld.add_action(camera_group)
    ld.add_action(mux_group)
    return ld
