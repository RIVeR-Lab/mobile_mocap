#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions as actions

from ament_index_python.packages import get_package_share_directory
from os.path import join
from yaml import safe_load
from os import system
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# Authored by Gary Lvov
def generate_launch_description():
    print(f"\n Spinning up, will be ready momentarily \n")
    
    first_camera_port = LaunchConfiguration('cam0_port')
    first_camera_port_arg = DeclareLaunchArgument(
        'cam0_port',
        default_value='4'
    )

    second_camera_port = LaunchConfiguration('cam1_port')
    second_camera_port_arg = DeclareLaunchArgument(
        'cam1_port',
        default_value='7'
    )

    ports = [int(first_camera_port), int(second_camera_port)]
    
    for port in ports:
        # although clunky, this ensures that the arducam's settings are correclty set.
        system(f"timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video{port} ! videoconvert ! autovideosink")

    assert ports is not None, "No USB cameras found"
    all_nodes = []
    calib_names = ["cam0_calib.yaml", "cam1_calib.yaml", "stereo_calib.yaml"]
    
    if ports == 1:
        calib_names = [calib_names[0], calib_names[-1]]

    camera_calib = [join(get_package_share_directory('mobile_mocap'), 
                    'camera_calibration', name) for name in calib_names]

    for idx, port in enumerate(ports):
        stream_param = None
        with open(camera_calib[idx]) as f:
            cam_param = safe_load(f)
        stream_param = {key: value for (key, value) in cam_param.items()}
        stream_param.update({"port": port, "rectify": True})

        nodes = [
                # Camera stream
                actions.Node(package="mobile_mocap", 
                            namespace="camera" + str(idx),
                            executable="publish_feed",
                            parameters=[stream_param]),
                
                # Finding markers
                actions.Node(package="mobile_mocap", 
                            namespace="camera" + str(idx),
                            executable="find_markers",
                            output="screen", 
                            emulate_tty=True, 
                            parameters=[{"frequency": 30, 
                                         "circularity_thresh": .7,
                                         "publish_debug_stream": True}])]
        all_nodes.extend(nodes)

    stereo_param = None
    with open(camera_calib[len(ports)]) as f:
        param = safe_load(f)

    stereo_param = {key: value for (key, value) in param.items()}
    stereo_param.update({"number_cameras": len(ports)})
    all_nodes.append(actions.Node(package="mobile_mocap",
        executable="triangulate", 
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'], # uncomment to debug
        parameters=[stereo_param]))
    
    all_nodes.append(
                # rigid body tracking
                actions.Node(package="mobile_mocap",
                     executable="rigid_body_tracker.py",
                     emulate_tty=True, 
                     output="screen"))

    print(f"\n All set, starting {len(all_nodes)} processes: \n")

    all_nodes.extend([first_camera_port_arg, second_camera_port_arg])
    return LaunchDescription(all_nodes) 