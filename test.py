import pyrealsense2 as rs
import numpy as np
import cv2
from robot import Robot
from multiprocessing import Process, Manager
from cam import Cam
import time

robot = Robot()
cam = Cam()
camX = 0
camY = 0
camZ = 0

while True:
    print("Start")
    robot.grab(float(300), float(240), float(380))
    time.sleep(1)
    """
    print("+X")
    robot.grab(float(420), float(240), float(450))
    time.sleep(1)
    robot.grab(float(320), float(240), float(450))
    time.sleep(1)
    print("-X")
    robot.grab(float(220), float(240), float(450))
    time.sleep(1)
    robot.grab(float(320), float(240), float(450))
    time.sleep(1)
    print("-Y")
    robot.grab(float(320), float(140), float(450))
    time.sleep(1)
    robot.grab(float(320), float(240), float(450))
    time.sleep(1)
    """
    print("+X")
    robot.grab(float(300), float(240), float(480))
    time.sleep(1)
    robot.grab(float(300), float(240), float(380))
    time.sleep(1)
"""
while True:
    print("Input cam coordinates for pen")
    camX = float(input("camX"))
    camY = float(input("camY"))
    camZ = float(input("camZ"))
    robot.grab(camX, camY, camZ)
"""