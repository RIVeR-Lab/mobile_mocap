#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mobile_mocap.srv import SetCams

import cv2 as cv
import numpy as np

import PySimpleGUI as sg

from os import mkdir, chdir, scandir, system, getcwd
import copy

# Authored by Gary Lvov

class Calibration(Node):
    def __init__(self):
        super().__init__("calibration")

        self.declare_parameter(name="square_size", value=40) # mm
        self.square_size = self.get_parameter('square_size').get_parameter_value().integer_value

        # self.cli = self.create_client(SetCams, 'setup_cameras')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')

        self.checkers = (4, 5)
        self.ports = [4, 8]
        self.create_folders()
        self.create_gui()
        
    def get_ports(self):
        req = SetCams.Request()
        req.adjust_values = False
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        ports = [x for x in res.ports]
        return ports

    def create_folders(self):
        try:
            mkdir("calibration_000")
        except FileExistsError:
            print("Calibrations exist in this directory, adding newest calibration in new folder")

        files = [x.path for x in scandir(".") if x.is_dir()]

        files = list(filter(lambda x: x[:13] == "./calibration",  files[::-1]))
        highest = max(files[-3:])
        highest = int(highest[-3:])
        next = highest + 1
        self.next_name = "calibration_000" + str(next)
        mkdir(self.next_name)
        chdir(self.next_name)
        mkdir("cam1")
        mkdir("cam2")
        chdir("../")

    def update_config(self):
        chdir("../") # Now in parent folder
        chdir("/scripts/matlab/")
        cam_dir = f"../../calibration/{self.next_name}/cam"
        config_dir = f"../../config/"
        matlab_single_cam = f"matlab -r 'add_path{getcwd()}; -nodisplay -nojvm try calibrateSingleCamera("
        mat_exit = "); catch; end; quit'"
        system(f"{matlab_single_cam}{cam_dir}1, {config_dir}cam1_calib.yaml, {self.square_size}{mat_exit}")
        system(f"{matlab_single_cam}{cam_dir}2, {config_dir}cam2_calib.yaml, {self.square_size}{mat_exit}")
        chdir("../../") # change back to parent folder

    def create_gui(self):
        img_scale = .5

        sg.theme("DarkPurple")

        layout = [[sg.Text("Gary's Sick Stereo Calibration Helper Tool", size=(40, 1), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='cam1'), sg.Image(filename='', key='cam2')],
              [sg.Button('Save Image', size=(10, 1), font='Helvetica 14'),]]

        window = sg.Window("Gary's Camera Calibration Helper Tool",
                        layout, location=(800, 400))

        assert len(self.ports) >= 2, "Not enough cameras located in camera search"
        cap1 = cv.VideoCapture(self.ports[0])
        cap2 = cv.VideoCapture(self.ports[1])
        _, frame1 = cap1.read()
        resize_dim =(int(frame1.shape[1]*img_scale), int(frame1.shape[0] * img_scale))
        saved_count = 0
        while rclpy.ok():
            _, frame1 = cap1.read()
            _, frame2= cap2.read()
           
            event, values = window.read(timeout=20)

            if event == 'Save Image':
                cv.imwrite(self.next_name + "/cam1/" + str(saved_count) + ".png", frame1)
                cv.imwrite(self.next_name + "/cam2/" + str(saved_count) + ".png", frame2)
                saved_count += 1
            
            elif event == sg.WIN_CLOSED:
                self.update_config()
                rclpy.shutdown()
                print("Updating configuration")
                break 

            resized_frame1 = cv.resize(frame1, resize_dim, cv.INTER_AREA)
            resized_frame2 = cv.resize(frame2, resize_dim, cv.INTER_AREA)
            imgbytes1 = cv.imencode('.png', resized_frame1)[1].tobytes() 
            imgbytes2 = cv.imencode('.png', resized_frame2)[1].tobytes()
            window['cam1'].update(data=imgbytes1)
            window['cam2'].update(data=imgbytes2)

def main(args=None):
    rclpy.init(args=args)
    calib = Calibration()
    rclpy.spin(calib)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
