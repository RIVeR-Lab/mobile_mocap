#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mobile_mocap.srv import SetCams
from subprocess import getoutput
from os import system, mkdir
from time import sleep
import cv2 as cv

# Authored by Gary Lvov
class SetupCameras(Node):
    def __init__(self):
        super().__init__('setup_cameras')
        self.srv = self.create_service(SetCams, "setup_cameras", self.service_callback)

    def service_callback(self, request, response):
        response.ports = SetupCameras.find_and_set_ports(request.adjust_values, request.names, request.values)
        return response    

    @staticmethod
    def find_and_set_ports(adjust_values=False, names=[], values=[]):
        list_cameras = getoutput("v4l2-ctl --list-devices").split("\n")
        camera_names = list(filter(lambda x: x[:3] == "USB", list_cameras))
        ports = [int(list_cameras[list_cameras.index(x)+1][len("/dev/video") + 1:]) for x in camera_names]
        if adjust_values:
            for port in ports:
                system(f"timeout 10 gst-launch-1.0 -v v4l2src device=/dev/video{port} ! videoconvert ! autovideosink")
                for (name, value) in zip(names, values):
                    system("v4l2-ctl --device=/dev/video" + str(port) + " --set-ctrl=" + name + "=" + str(value))
        return ports  

def main(args=None):
    rclpy.init(args=args)
    setup_cam = SetupCameras()
    rclpy.spin(setup_cam)
    rclpy.shutdown()

if __name__ == '__main__':
    main()