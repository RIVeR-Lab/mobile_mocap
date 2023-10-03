# Mobile Motion Capture

Welcome to the Mobile Motion Capture World. Developed with ROS 2 Humble/Ubuntu 22.04, compatible with ROS 2 Foxy / Ubuntu 20.04.

# Citation
```
@INPROCEEDINGS{10260562,
  author={Lvov, Gary and Zolotas, Mark and Hanson, Nathaniel and Allison, Austin and Hubbard, Xavier and Carvajal, Michael and Padir, Taşkin},
  booktitle={2023 IEEE 19th International Conference on Automation Science and Engineering (CASE)}, 
  title={Mobile MoCap: Retroreflector Localization On-The-Go}, 
  year={2023},
  volume={},
  number={},
  pages={1-7},
  doi={10.1109/CASE56687.2023.10260562}}
```

# Installation 
```
git clone git@github.com:RIVeR-Lab/mobile_mocap.git # in your colcon workspace

# If you're using ROS 2 Foxy and Ubuntu 20.04, run 'git checkout foxy'
sudo apt-get install v4l-utils
pip install opencv-python # Must be done AFTER ROS 2 install!
pip install scikit-spatial
sudo apt-get install ros-humble-image-common

pip install PySimpleGui # For calibration tool only
sudo apt-get install xterm # only needed for debugging
```

# Running Mobile MoCap
First, plug in the first camera of the stereo system. Determine the port number with
```
v4l2-ctl --list-devices
```
Then, plug in the second camera, and determine the port number again.

Adjust ```config/camera_calibration/cam0_calib.yaml``` to match your first camera intrinsic (camera matrix & distortion coefficents).

Adjust ```config/camera_calibration/cam1_calib.yaml``` to match your second camera intrinsic (camera matrix & distortion coefficents).

Adjust ```config/camera_calibration/stereo_calib.yaml``` to match your stereo camera intrinsic (projection matrices for both cameras).

Adjust the port values in ```config/camera_calibration/stereo_calib.yaml``` to match that determined with v4l2. It is important that camera0 and camera1 ports are always consistent (for example, camera 0 can always refer to the left camera).

Rebuild and resource your colcon workspace.
```
cd ~/YOUR_COLCON_WS/
colcon build
source install/setup.bash
```

```
ros2 launch mobile_mocap arducam_bringup.py
```

# Calibration
In order to determine the calibration parameters in ```config/camera_calibration/```, determine the camera matrix and distortion coefficents for both cameras.
Then, determine the translation and rotation between the two cameras. Fill in the parameters in ```scripts/determine_calib_param.py``` to compute the contents
of the calibation folder.

Calibration of the two cameras can be achieved using the [stereo calibration tool](https://github.com/sourishg/stereo-calibration)
