#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions as actions

from scripts.setup_cameras import SetupCameras

# Authored by Gary Lvov
def generate_launch_description():
    all_nodes = [actions.Node(package="mobile_mocap", executable="setup_cameras.py"), 
                 actions.Node(package="mobile_mocap", executable="calibrate.py")]
    return LaunchDescription(all_nodes) 