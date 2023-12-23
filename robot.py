import math
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
robot.gripper.set_pressure(0.8)

class Robot:
    def __init__(self, offsetX = 100, offsetY = 350, offsetZ = 350, offsetRot = math.pi/2, L1 = math.sqrt(100**2 + 35**2), L2 = 100, L3 = (86.05 + 129.15)/2, thetaOffset = math.atan(35/100)):
        #offsets between camera center and center of robot base, XYZ values provided in mm, offset Pitch/Roll/Yaw in radians
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.offsetZ = offsetZ
        self.offsetRot = offsetRot
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.thetaOffset = thetaOffset
        self.calX = 150
        self.calZ = 100
    
    def calculate_offset(self, camX, camY, camZ, penLength):
        # place the pen in the robot gripper to calculate offset between robot position and camera position. robot should be 90 degrees LEFT relative to camera. 
        # robot should hold bottom end of pen with majority of pen body above gripper
        a = -self.calX  + camX #use with camX
        b = camZ #use with camZ
        c = self.calZ + camY + penLength/2
        self.offsetX = a
        self.offsetY = b
        self.offsetZ = c
        print([a,b,c])

    def prepare_to_grab(self, camX, camY, camZ):
        #takes CAMERA coordinates as inputs, not robot
        xPos = camX - self.offsetX
        yPos = self.offsetY - camZ
        zPos = self.offsetZ - camY
        radius, zPos, thetaPos = self._cart_to_radial(xPos, yPos, zPos)
        retraction = 50
        radius += -retraction
        robot.gripper.release()
        move = self._move_to_radial_pos(radius, zPos, thetaPos)
        print("Preparing to grab at " + str([radius, zPos, thetaPos]) + " (radial).")
        if move == False:
            print("Not able to complete requested movement.")
        return move

    def grab(self, camX, camY, camZ):
        #takes CAMERA coordinates as inputs, not robot
        xPos = camX - self.offsetX
        yPos = self.offsetY - camZ
        zPos = self.offsetZ - camY
        radius, zPos, thetaPos = self._cart_to_radial(xPos, yPos, zPos)
        move = self._move_to_radial_pos(radius, zPos, thetaPos)
        robot.gripper.grasp()
        print("Grabbing pen at " + str([radius, zPos, thetaPos]) + " (radial).")
        if move == False:
            print("Not able to complete requested movement.")
        return move
        
    def initial_pose(self):
        self._move_to_radial_pos(50, 100, 0)

    def drop_pose(self):
        self._move_to_radial_pos(200, 50, 3 *math.pi/4)

    def calibration_pose(self):
        self._move_to_radial_pos(self.calX, self.calZ, 0)

    def open_gripper(self):
        robot.gripper.release()

    def close_gripper(self):
        robot.gripper.grasp()

    def _cart_to_radial(self, xPos, yPos, zPos):
        print("Pen position in robot coordinate system: " + str([xPos, yPos, zPos]) + " (cart).")
        if yPos < 0 :
            thetaPos = math.atan(xPos/yPos) + self.offsetRot
        else:
            thetaPos = math.atan(xPos/yPos) + self.offsetRot - math.pi
        radius = math.sqrt(xPos**2 + yPos**2)
        return [radius, zPos, thetaPos]

    def _move_to_radial_pos(self, radius, zPos, thetaPos):
        zC = zPos
        xC = radius - self.L3
        Z = math.dist([xC,zC], [0,0])
        H2 = self.L1**2 - ((Z**2 - self.L2**2 + self.L1**2)/(2 * Z))**2
        if H2 > 0:
            H = math.sqrt(H2)
            thetaB = math.pi/2 - math.acos(H/self.L1) - math.acos(H/self.L2) + self.thetaOffset
            thetaA = math.pi/2 - math.acos(xC/Z) - math.asin(H/self.L1) - self.thetaOffset
            thetaC = - thetaA - thetaB
            robot.arm.set_joint_positions([thetaPos, thetaA, thetaB, thetaC])
            return [thetaA, thetaB, thetaC]
        else:
            return False

