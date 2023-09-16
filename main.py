import pyrealsense2 as rs
import numpy as np
import cv2
from robot import Robot
from multiprocessing import Process, Manager
from cam import Cam
import time

robot = Robot()
cam = Cam()

#create pipeline and config
pipeline = rs.pipeline()
config = rs.config()
pen_visible = False

# Get device product line for setting a supporting resolution
#pipeline_wrapper = rs.pipeline_wrapper(pipeline)
#pipeline_profile = config.resolve(pipeline_wrapper)
#device = pipeline_profile.get_device()
#device_product_line = str(device.get_info(rs.camera_info.product_line))

#enable depth and color stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#start stream
profile = pipeline.start(config)

#create align object
align_target = rs.stream.color
align = rs.align(align_target)

robot.initial_pose()
robot.open_gripper()

check = "s"

while check != "q":
    check = input("Enter [s] to begin search for pen. Enter [c] to calibrate robot and camera system. Enter [q] to quit.")
    if check == "s":
        print("Searching for pen...")
        cx,cy,cz, pen_visible = cam.find_pen(pipeline, align)
        if pen_visible == True:
            print("Pen detected at " + str([cx,cy,cz]) + " (cart).")
            print("Preparing to grab Pen.")
            prep_check = robot.prepare_to_grab(cx,cy,cz)
            if prep_check != False:
                time.sleep(1)
                print("Grabbing Pen.")
                robot.grab(cx,cy,cz)
                robot.drop_pose()
                robot.open_gripper()
                robot.initial_pose()
            else:
                print("Could not reach pen!")
    elif check == "c":
        robot.calibration_pose()
        time.sleep(1)
        robot.open_gripper()
        input("Press [enter] to close the gripper...")
        robot.close_gripper()
        penLength = input("Enter length of pen to run calibration: ")
        cx,cy,cz, pen_visible = cam.find_pen(pipeline, align)
        if pen_visible == True:
            robot.calculate_offset(float(cx),float(cy),float(cz), float(penLength))
        time.sleep(1)
        print("Calibration complete!")
        robot.drop_pose()
        robot.open_gripper()
        robot.initial_pose()
    elif check == "q":
        break
    else:
        print("Invalid entry.")
        
                
pipeline.stop()