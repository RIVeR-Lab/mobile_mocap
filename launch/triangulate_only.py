#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions as actions

from scripts.setup_cameras import SetupCameras

from ament_index_python.packages import get_package_share_directory
from os.path import join
from yaml import safe_load
from time import sleep
# Authored by Gary Lvov


def generate_launch_description():
    all_nodes = []
    ports = [1, 2]
    calib_names = ["cam0_calib.yaml", "cam1_calib.yaml", "stereo_calib.yaml"]
    camera_calib = [join(get_package_share_directory('mobile_mocap'),
                    'camera_calibration', name) for name in calib_names]
    stereo_param = None
    with open(camera_calib[len(ports)]) as f:
        param = safe_load(f)

    stereo_param = {key: value for (key, value) in param.items()}
    stereo_param.update({"number_cameras": len(ports)})
    print(stereo_param)

    all_nodes.append(actions.Node(package="mobile_mocap",
                                  executable="triangulate",
                                  output="screen",
                                  # prefix=['xterm -e gdb -ex run --args'], # uncomment to debug
                                  parameters=[stereo_param]))
    
    all_nodes.append( actions.Node(package="mobile_mocap",
                     executable="rigid_body_tracker.py",
                     output="screen"))

    print(f"\n All set, starting {len(all_nodes)} processes: \n")
    return LaunchDescription(all_nodes)
