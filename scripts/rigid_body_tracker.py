#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from mobile_mocap.msg import TriangulatedMarkers
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from skspatial.objects import Plane, Points
from mathutils import Quaternion
import math
import numpy as np

# Authored by Gary Lvov
class RigidBodyTracker(Node):
    def __init__(self):
        super().__init__('rigid_body_tracker')
        self.declare_parameter(name="body_names", value=["cal_cube"])
        self.declare_parameter(name="body_descriptions_lengths", value=[3])
        # tracking of a scalene triangle (geometry param in meters)
        self.declare_parameter(name="body_description_locations_x",
                               value=[-.05, -.05, .05])
        self.declare_parameter(name="body_description_locations_y",
                               value=[.05, -0.02, -.05])

        self.declare_parameter(
            name="body_description_locations_z", value=[0.0, 0.0, 0.0])

        self.initialize_param()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.feed_sub = self.create_subscription(
            TriangulatedMarkers, "triangulated_markers", self.markers_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)

    def rigid_transform_3D(self, A, B):
        # https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
        if not A.shape == B.shape:
            return None, None

        num_rows, num_cols = A.shape
        if num_rows != 3:
            raise Exception(
                f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

        num_rows, num_cols = B.shape
        if num_rows != 3:
            raise Exception(
                f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

        # find mean column wise
        centroid_A = np.mean(A, axis=1)
        centroid_B = np.mean(B, axis=1)

        # ensure centroids are 3x1
        centroid_A = centroid_A.reshape(-1, 1)
        centroid_B = centroid_B.reshape(-1, 1)

        # subtract mean
        Am = A - centroid_A
        Bm = B - centroid_B

        H = Am @ np.transpose(Bm)

        # # sanity check
        # if np.linalg.matrix_rank(H) < 3:
        #    raise ValueError("rank of H = {}, expecting 3".format(np.linalg.matrix_rank(H)))

        # find rotation
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # special reflection case
        if np.linalg.det(R) < 0:
            # print("det(R) < R, reflection detected!, correcting for it ...")
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        t = -R @ centroid_A + centroid_B

        return R, t

    def describe_geometry(self, points):
        all_geometries = []
        
        for idx, point1 in enumerate(points):
            max_magnitude = -1
            relative_geometry = []
            for jdx, point2 in enumerate(points):
               
                if jdx != idx:
                    edge = Edge(point1, point2)
                    relative_geometry.append(edge)

                    if edge.magnitude > max_magnitude:
                        max_magnitude = edge.magnitude
            
            geom, mag = (sorted(relative_geometry, key=lambda x: x.magnitude), max_magnitude)
            all_geometries.append((geom, mag))
        
        all_geometries = sorted(all_geometries, key=lambda x:x[1])

        return np.array([x[0] for x in all_geometries])

    def fit_geometry_to_sparse_pointcloud(self, pc_geos, rigid_body_geos):
        for jdx, rigid_body_geo in enumerate(rigid_body_geos):
            

            shortest_edge = rigid_body_geo[jdx][0][0]
            longest_edge = rigid_body_geo[jdx][0][len(rigid_body_geo[jdx][0]) - 1]

            minimizing_edges = {"pot_shortest_idx": None, "pot_longest_idx": None, "error": float(
                "inf"), "pc_geo_num": 0}

            for idx, pc_geo in enumerate(pc_geos):
                length = len(pc_geo)
                for shortest_guess_idx in range(length):
                    for longest_guess_idx in range(length - 1, 0, -1):
                        shortest_guess = pc_geo[shortest_guess_idx]
                        longest_guess = pc_geo[longest_guess_idx]
                        error = Edge.matching_error(shortest_guess, longest_guess, shortest_edge, longest_edge)

                        if error < minimizing_edges["error"]:
                            minimizing_edges["pot_shortest_idx"] = shortest_guess_idx
                            minimizing_edges["pot_longest_idx"] = longest_guess_idx
                            minimizing_edges["error"] = error
                            minimizing_edges["pc_geo_num"] = idx

            min_error_idx = minimizing_edges['pc_geo_num']
            rigid_body = np.array([rigid_body_geo[jdx][0][0].point2,
                                   rigid_body_geo[jdx][0][1].point2,
                                   rigid_body_geo[jdx][0][0].point1])

            correspondence_guess = np.array([pc_geos[min_error_idx][minimizing_edges["pot_shortest_idx"]].point2,
                                             pc_geos[min_error_idx][minimizing_edges["pot_longest_idx"]].point2,
                                             pc_geos[min_error_idx][minimizing_edges["pot_longest_idx"]].point1])
            name = rigid_body_geo[1]

            rot, t = self.rigid_transform_3D(rigid_body.T, correspondence_guess.T)
            self.publish_pose(rot, t, object_frame=name)

    def publish_pose(self, rot, tvec, object_frame="default"):
        rot = R.from_matrix(rot)
        rot = rot.as_quat()

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera0"
        transform.child_frame_id = object_frame
        transform.transform.rotation.x = float(rot[0])
        transform.transform.rotation.y = float(rot[1])
        transform.transform.rotation.z = float(rot[2])
        transform.transform.rotation.w = float(rot[3])
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])
        self.tf_broadcaster.sendTransform(transform)

    def markers_callback(self, msg):
        points = []

        for point in msg.points:
            points.append([point.x, point.y, point.z])
        points = np.array(points)

        geo = None
        if len(points) >= 3:
            geo = self.describe_geometry(points)
        if geo is not None:
            correspondence = self.fit_geometry_to_sparse_pointcloud(geo, self.rigid_bodies)

    def initialize_param(self):
        names = self.get_parameter(
            'body_names').get_parameter_value().string_array_value
        lengths = self.get_parameter(
            'body_descriptions_lengths').get_parameter_value().integer_array_value
        xs = self.get_parameter(
            'body_description_locations_x').get_parameter_value().double_array_value
        ys = self.get_parameter(
            'body_description_locations_y').get_parameter_value().double_array_value
        zs = self.get_parameter(
            'body_description_locations_z').get_parameter_value().double_array_value

        self.rigid_bodies = []

        for name, length in zip(names, lengths):
            points = []
            for idx in range(length):
                points.append([xs[idx], ys[idx], zs[idx]])

            geometry = self.describe_geometry(np.array(points))
            self.rigid_bodies.append((self.describe_geometry(
                np.array(points)), name))


class Edge():
    def __init__(self, point1, point2):
        self.magnitude = np.linalg.norm(point2 - point1)
        self.unit_vector = (point2 - point1) / self.magnitude
        self.point1 = point1
        self.point2 = point2

    @staticmethod
    def determine_angle(first: type[Edge], other: type[Edge]) -> np.float32:
        return np.arccos(np.dot(first.unit_vector, other.unit_vector))

    @staticmethod
    def matching_error(pot_first: type[Edge], pot_second, first: type[Edge], second: type[Edge]) -> np.float32:
        pot_angle = Edge.determine_angle(pot_first, pot_second)
        angle = Edge.determine_angle(first, second)

        pot_x = pot_second.magnitude * math.cos(pot_angle)
        pot_y = pot_second.magnitude * math.sin(pot_angle)

        x = second.magnitude * math.cos(angle)
        y = second.magnitude * math.sin(angle)

        return math.sqrt(math.pow(pot_x - x, 2) + math.pow(pot_y - y, 2))

    def __str__(self):
        return f"Edge, starting at {self.point1}, ending at {self.point2}, with magnitude {self.magnitude}"
    
    def __eq__(self, other):
        return self.point1 == other.point1 and self.point2 == other.point2


def main(args=None):
    rclpy.init(args=args)
    rbt = RigidBodyTracker()
    rclpy.spin(rbt)
    rclpy.shutdown()


if __name__ == '__main__':
    main()