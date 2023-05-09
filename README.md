# Mobile Motion Capture
Welcome to the Mobile Motion Capture World. Developed with ROS 2 Humble/Ubuntu 22.04, compatible with ROS 2 Foxy / Ubuntu 20.04.

# Citation
```
@article{lvov2023momocap,
  title={Mobile MoCap: Retroreflector Localization On-The-Go},
  author={Lvov, Gary and Zolotas, Mark and Hanson, Nathaniel and Allison, Austin and Hubbard, Xavier and Carvajal, Michael and Padir, Taskin},
  journal={arXiv preprint arXiv:2303.13681},
  year={2023},
  url={https://arxiv.org/abs/2303.13681},
  doi={10.48550/arXiv.2303.13681},
}
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
```
ros2 launch mobile_mocap arducam_bringup.py cam0_port:=XXX cam1_port:=XXX
```

# Calibration
In order to determine the calibration parameters in ```config/camera_calibration/```, determine the camera matrix and distortion coefficents for both cameras.
Then, determine the translation and rotation between the two cameras. Fill in the parameters in ```scripts/determine_calib_param.py``` to compute the contents
of the calibation folder.

Calibration of the two cameras can be achieved using the [stereo calibration tool](https://github.com/sourishg/stereo-calibration)
