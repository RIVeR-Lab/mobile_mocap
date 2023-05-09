#!/usr/bin/env python3
import numpy as np
# Authored by Gary Lvov and Xavier Hubbard

#  Output from Matlab stereo calibration tool, from 

# [intrinsicMatrix1,distortionCoefficients1,intrinsicMatrix2, ...
#    distortionCoefficients2,rotationOfCamera2,translationOfCamera2] =...
#    stereoParametersToOpenCV(stereoParams)

# intrinsicMatrix1 =
# .26 pixel reprojection errror
#   582.6918         0  321.2192
#          0  582.2707  252.7513
#          0         0    1.0000


# distortionCoefficients1 =

#    -0.4538    0.2985    0.0011   -0.0004   -0.1391


# intrinsicMatrix2 =

#   584.9188         0  317.6645
#          0  584.5984  239.9013
#          0         0    1.0000


# distortionCoefficients2 =

#    -0.4514    0.2970    0.0019   -0.0016   -0.1400


# rotationOfCamera2 =

#     0.9999    0.0022   -0.0113
#    -0.0021    0.9999    0.0100
#     0.0113   -0.0100    0.9999


# translationOfCamera2 =

#   -95.9733    0.9481   -0.8270

# camera 1 matrix
K1 = np.array([
[582.3094 , 0, 323.6991],
[0, 581.8881, 258.2891],
[0, 0, 1.0000],
])

# camera 2 matrix
K2 = np.array([
[585.9528, 0, 315.2594],
[0, 586.0343, 245.7776],
[0, 0, 1.0000],
])

# rotation between camera 1 and camera 1. Should be identity, do not modify
R1 = np.eye(3)

# rotation between camera 1 and camera 2.
R2 = np.array([
[1.0000, 0.0013, -0.0030],
[-0.0012, 0.9999, 0.0107],
[0.0030, -0.0107, 0.9999],
])

# translation between camera 1 and camera 1, should be 0, do not modify
t1 = np.array([[0.], [0.], [0.]])

# translation between camera 1 and camera 2 in millimeters
t2 = np.array([[-96.7144], [.3070], [2.3368]]) * 0.001 # convert to meters

# distortion coefficients for camera 1
distCoeffs1 = np.array([-0.4538, 0.2985, 0.0011, -0.0004, -0.1391])

# distortion coefficients for camera 2
distCoeffs2 = np.array([-0.4514, 0.2970, 0.0019, -0.0016, -0.1400])

# do not modify the following, computes the projection matrices
P1 = np.hstack([R1, R1.dot(t1)])
P2 = np.hstack([R2, R2.dot(t2)])

P1 = K1.dot(P1)
P2 = K2.dot(P2)
np.set_printoptions(suppress=True)

print('-----------------------------')
print('cam0_calib.yaml')
print(f"camera_matrix_flattened: {[x for x in K1.flatten()]}")
print(f"camera_distortion_coefficients: {[x for x in distCoeffs1.flatten()]}")

print('-----------------------------')

print('cam1_calib.yaml')

print(f"camera_matrix_flattened: {[x for x in K2.flatten()]}")
print(f"camera_distortion_coefficients: {[x for x in distCoeffs2.flatten()]}")

print('-----------------------------')
print('stereo_calib.yaml')
print(f"camera0_projection_matrix_flattened: {[x for x in P1.flatten()]}")
print(f"camera1_projection_matrix_flattened: {[x for x in P2.flatten()]}")
print('-----------------------------')