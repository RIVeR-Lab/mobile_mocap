#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions as actions

from ament_index_python.packages import get_package_share_directory
from os.path import join
from yaml import safe_load
from os import system

# Authored by Gary Lvov
def generate_launch_description():
    print(f"\n Spinning up, will be ready momentarily \n")

    all_nodes = []
    calib_names = ["cam0_calib.yaml", "cam1_calib.yaml", "stereo_calib.yaml"]
    camera_calib = [join(get_package_share_directory('mobile_mocap'),
                    'camera_calibration', name) for name in calib_names]

    stereo_param = None
    with open(camera_calib[2]) as f:
        stereo_param = safe_load(f)

    ports = [stereo_param['camera0_port'], stereo_param['camera1_port']]
    print(ports[0])
    print(ports[1])

    for port in ports:
        # although clunky, this ensures that the arducam's settings are correclty set.
        system(
            f"timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video{port} ! videoconvert ! autovideosink")

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

    return LaunchDescription(all_nodes)
