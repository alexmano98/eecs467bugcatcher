import numpy as np
import cv2
from rexarm import Rexarm
from trajectory_planner import TrajectoryPlanner
import time
from copy import deepcopy

from picamera.array import PiRGBArray
from picamera import PiCamera
from apriltags3 import Detector

import os
os.sys.path.append('dynamixel/')  # Path setting

from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

D2R = 3.141592 / 180.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0".encode('utf-8')

camera_params = [604.37968515, 603.92764597, 325.93761715, 241.48766872]  # [fx, fy, cx, cy]
extrinsic = np.array([[-0.02963398, -0.30054063, 0.95330853, 0.01931047],
                       [-0.99810848, 0.06029093, -0.01201925, 0.00316656],
                       [-0.05386358, -0.95186151, -0.30175881, 0.01349531],
                       [0., 0., 0., 1.]])


def detect_apriltag(detector, gray_image):
    tags = detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.0127)
    for tag in tags:
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                         [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                         [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                         [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0, 1]))
        arm_pose = np.matmul(extrinsic, tag_pose)
    return tags, arm_pose


def main():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    # Wait for the automatic gain control to settle
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.awb_gains = (327 / 256, 425 / 256)
    rawCapture = PiRGBArray(camera, size=(640, 480))
    dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
    port_num = dxlbus.port()
    base = DXL_XL(port_num, 1)
    shld = DXL_XL(port_num, 2)
    elbw = DXL_XL(port_num, 3)
    wrst = DXL_XL(port_num, 4)
    grip = DXL_XL(port_num, 5)
    rexarm = Rexarm((base, shld, elbw, wrst, grip))
    rexarm.set_speeds_normalized_global(25 / 100.0)
    tp = TrajectoryPlanner(rexarm)
    detector = Detector("tagStandard41h12", quad_decimate=2.0, quad_sigma=1.0, debug=False)
    rexarm.set_positions([0, 0, 0, 0, 0])
    time.sleep(2)
    while True:
        for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
            image = frame.array
            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            tags, arm_pose = detect_apriltag(detector, gray_image)
            print("Arm pose: ", arm_pose)
            if len(tags) > 0:
                rexarm.get_feedback()
                rexarm.set_positions([0, 0, -85 * D2R, 0, 0]) # give home pos
                time.sleep(2)
                rexarm.get_feedback()
                print("Fb: " + str(rexarm.joint_angles_fb))
                ik_solve = rexarm.rexarm_FK()
                print("ik solve: " + str(ik_solve))
                time.sleep(2)
                rexarm.set_positions([0, 0, 0, 0, 0])
                time.sleep(2)
                rexarm.get_feedback()

                arm_pose[-1] = 90
                print(arm_pose)
                tp.set_initial_wp()
                tp.set_final_wp(arm_pose)
                rexarm.open_gripper()
                time.sleep(2)
                prev_joint = None
                for joint in tp.go():
                    if prev_joint:
                        print("Waiting for feedback to stabilize")
                        changed = False
                        while changed:
                            changed = False
                            for i, j in enumerate(joint):
                                if joint[i] != prev_joint[i]:
                                    print(joint[i], " != ", prev_joint[i])
                                    changed = True
                    prev_joint = deepcopy(joint)
                    print("joint: ", joint)
                    joint[-1] = np.deg2rad(-60.0)
                    for i,v in enumerate(joint):
                        rexarm.position[i] =  v
                    rexarm.send_commands()
                    time.sleep(0.5)
                    rexarm.get_feedback()
                rexarm.close_gripper()
                print("done")
                exit()


main()
