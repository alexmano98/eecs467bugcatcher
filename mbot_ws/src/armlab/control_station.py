#!/usr/bin/env python
import lcm
import sys
import cv2
import numpy as np
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)
import qimage2ndarray

import os
import io

os.sys.path.append('dynamixel/')  # Path setting
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

from ui import Ui_MainWindow
from rexarm import Rexarm
from trajectory_planner import TrajectoryPlanner
from state_machine import StateMachine

from picamera.array import PiRGBArray
from picamera import PiCamera
from apriltags3 import Detector

from task1 import task1
from task2 import task2
from task3 import task3
from task4 import task4
from driver import driver
""" Radians to/from  Degrees conversions """
D2R = 3.141592 / 180.0
R2D = 180.0 / 3.141592
# pixel Positions of image in GUI
MIN_X = 240
MAX_X = 880
MIN_Y = 40
MAX_Y = 520

""" Serial Port Parameters"""
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0".encode('utf-8')

"""Threads"""


class VideoThread(QThread):
    # TODO: You can make updateFrame take more images in the list as input.
    # That way you will be emit for images to visualize for debugging.
    updateFrame = pyqtSignal(list)
    updateAprilTags = pyqtSignal(list)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        # Can be set to 1920 x 1080, but it will be slow. 
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        # Wait for the automatic gain control to settle
        time.sleep(2)
        # turn off auto exposure and white balance,
        # otherwise RBG/HSV will be changing for the same object
        self.camera.shutter_speed = camera.exposure_speed
        self.camera.exposure_mode = 'off'
        g = self.camera.awb_gains
        # print("AWB gains: ", g)
        self.camera.awb_mode = 'off'
        # TODO: Use fixed awb_gains values instead of
        # using some random initialized value.
        self.camera.awb_gains = (327 / 256, 425 / 256)   # (red, blue) tuple, usually between (0.9, 1.9)
        self.color = 'none'
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.detector = Detector("tagStandard41h12", quad_decimate=2.0, quad_sigma=1.0, debug=False)
        # TODO: load your camera parameters here. These camera parameters are intrinsics.
        self.camera_params = [604.37968515, 603.92764597, 325.93761715, 241.48766872]  # [fx, fy, cx, cy]
        self.intrinsic = np.array([[604.37968515, 0, 325.93761715], [0, 603.92764597, 241.48766872], [0, 0, 1]])


    def run(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True):
            image = frame.array
            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            # TODO: Disable apriltag detection if it is slow and unnecessary to run
            # each time in the loop. Move it to the state machine and do the detection
            # only when necessary.
            tags, apriltag_image = self.detect_apriltag(gray_image)

            block_image = self.detect_block(image)
            self.updateFrame.emit([image, apriltag_image, block_image])
            self.updateAprilTags.emit(tags)
            self.rawCapture.truncate(0)

    def detect_apriltag(self, gray_image):
        tags = self.detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params,
                                    tag_size=0.0127)
        # print("=============================")
        # print(tags)
        # visualize the detection
        apriltag_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(apriltag_image, tuple(tag.corners[idx - 1, :].astype(int)),
                         tuple(tag.corners[idx, :].astype(int)), (255, 0, 0))

            # label the id of AprilTag on the image.
            cv2.putText(apriltag_image, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1,
                        color=(255, 0, 0))

        return tags, apriltag_image

    def detect_block(self, rgb_image):
        # Detect a block based on the range of color space values.
        # An example of detecting green color block.
        # TODO: You can modify and use this block detector to make your color detection/identification
        # more robust. You can identify the center of the detected color block, and filter out the
        # blocks whose center is too far away from the AprilTag's pixel coordinates.
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        # thresholds for green color in hsv space. You may need to tune this.
        # Note: You will need to avoid underflow or overflow uint8.
        ranges = []
        # green
        ranges.append([np.uint8([60, 200, 30]), np.uint8([90, 255, 255])])
        # blue
        ranges.append([np.uint8([100, 150, 30]), np.uint8([125, 255, 255])])
        # purple
        ranges.append([np.uint8([125, 50, 30]), np.uint8([155, 255, 255])])
        ranges.append([np.uint8([155, 50, 30]), np.uint8([180, 125, 125])])
        # orange
        ranges.append([np.uint8([0, 150, 50]), np.uint8([15, 255, 255])])
        # yellow
        ranges.append([np.uint8([20, 170, 80]), np.uint8([45, 255, 255])])
        # red
        ranges.append([np.uint8([160, 130, 130]), np.uint8([185, 255, 255])])
        ims = []
        sums = []
        for r in ranges:
            # Find the mask for the pixels whose color is in the range.
            mask = cv2.inRange(hsv_image, r[0], r[1])
            # maks the original rgb image.
            im = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)
            sums.append(np.sum(im > 0))
            ims.append(im)
        color_ind = np.argmax(sums)
        if color_ind == 0 or color_ind == 4 or color_ind == 6:
            self.color = 'trash_block'
        else:
            self.color = 'normal_block'
        mask_image = ims[0] + ims[1] + ims[2] + ims[3] + ims[4] + ims[5] + ims[6]

        # read more from the following link to know Morphological Transformations
        # https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        open_kernel = np.ones([5, 5], np.uint8)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, open_kernel)
        close_kernel = np.ones([5, 5], np.uint8)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_CLOSE, close_kernel)

        return mask_image


class LogicThread(QThread):
    # Run the lowerest level state machine.
    def __init__(self, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.sm = state_machine

    def run(self):
        while True:
            self.sm.run()
            time.sleep(0.05)


class TaskThread(QThread):
    # Run the tasks which can be decomposed into combinations
    # of lowest level states. Each task itself is a state machine.
    # TODO: implement state machines for complex tasks.
    def __init__(self, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.task_num = 9
        self.driver = driver(state_machine)
        self.task1 = task1(state_machine)
        self.task2 = task2(state_machine)
        self.task3 = task3(state_machine)
        self.task4 = task4(state_machine)
    def run(self):
        while True:
            if self.task_num == 0:
                self.driver.operate_task()
            elif self.task_num == 1:
                self.task1.operate_task()
                self.task_num = 5
            elif self.task_num == 2:
                self.task2.operate_task()
                self.task_num = 5
            elif self.task_num == 3:
                self.task3.operate_task()
                self.task_num = 5
            elif self.task_num == 4:
                self.task4.operate_task()
                self.task_num = 5
            time.sleep(0.05)

    def set_task_num(self, task_num):
        self.task_num = task_num
        print("task change: ", self.task_num)
        if self.task_num == 1:
            self.task1.begin_task()
        elif self.task_num == 2:
            self.task2.begin_task()
        elif self.task_num == 3:
            self.task4.begin_task()
        elif self.task_num == 4:
            self.task4.begin_task()


class DisplayThread(QThread):
    updateStatusMessage = pyqtSignal(str)
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, rexarm, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.rexarm = rexarm
        self.sm = state_machine

    def run(self):
        while True:
            self.updateStatusMessage.emit(self.sm.status_message)
            self.updateJointReadout.emit(self.rexarm.joint_angles_fb)
            self.updateEndEffectorReadout.emit(list(self.rexarm.rexarm_FK()))
            time.sleep(0.1)


"""GUI Class"""


class Gui(QMainWindow):
    """
    Main GUI Class
    contains the main function and interfaces between
    the GUI and functions
    """

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Set GUI to track mouse """
        QWidget.setMouseTracking(self, True)

        """
        Dynamixel bus
        TODO: add other motors here as needed with their correct address"""
        self.dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
        port_num = self.dxlbus.port()
        base = DXL_XL(port_num, 1)
        shld = DXL_XL(port_num, 2)
        elbw = DXL_XL(port_num, 3)
        wrst = DXL_XL(port_num, 4)
        grip = DXL_XL(port_num, 5)

        """Objects Using Other Classes"""
        self.camera = PiCamera()
        self.rexarm = Rexarm((base, shld, elbw, wrst, grip))
        self.tp = TrajectoryPlanner(self.rexarm)
        self.sm = StateMachine(self.rexarm, self.tp)
        self.taskThread = TaskThread(self.sm)
        self.rgb_image = None

        """ 
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(self.sm.set_current_state, "calibrate"))

        self.ui.btn_task1.clicked.connect(partial(self.taskThread.set_task_num, 0))
        self.ui.btn_task2.clicked.connect(partial(self.taskThread.set_task_num, 2))
        self.ui.btn_task3.clicked.connect(partial(self.taskThread.set_task_num, 3))
        self.ui.btn_task4.clicked.connect(partial(self.taskThread.set_task_num, 4))
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)

        self.ui.sldrWrist2.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)

        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        self.ui.rdoutStatus.setText("Waiting for input")

        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """initalize rexarm"""
        self.rexarm.initialize()

        """Setup Threads"""
        self.videoThread = VideoThread(self.camera)
        self.videoThread.updateFrame.connect(self.setImage)
        self.videoThread.updateAprilTags.connect(self.updateAprilTags)
        self.videoThread.start()

        self.logicThread = LogicThread(self.sm)
        self.logicThread.start()

        self.taskThread.start()

        self.displayThread = DisplayThread(self.rexarm, self.sm)
        self.displayThread.updateJointReadout.connect(self.updateJointReadout)
        self.displayThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
        self.displayThread.updateStatusMessage.connect(self.updateStatusMessage)
        self.displayThread.start()

        """ 
        Setup Timer 
        this runs the trackMouse function every 50ms
        """
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.trackMouse)
        self._timer.start(50)

    """ Slots attach callback functions to signals emitted from threads"""

    # Convert image to Qt image for display.
    def convertImage(self, image):
        qimg = QImage(image, image.shape[1], image.shape[0], QImage.Format_RGB888)
        return QPixmap.fromImage(qimg)

    # TODO: Add more QImage in the list for visualization and debugging
    @pyqtSlot(list)
    def setImage(self, image_list):
        rgb_image = image_list[0]
        apriltag_image = image_list[1]
        block_image = image_list[2]
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(self.convertImage(rgb_image))
            self.rgb_image = rgb_image
        if (self.ui.radioApril.isChecked()):
            self.ui.videoDisplay.setPixmap(self.convertImage(apriltag_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(self.convertImage(block_image))

    @pyqtSlot(list)
    def updateAprilTags(self, tags, color=None):
        self.sm.tags = tags
        self.sm.block_type = self.videoThread.color

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        self.ui.rdoutBaseJC.setText(str("%+.2f" % (joints[0] * R2D)))
        self.ui.rdoutShoulderJC.setText(str("%+.2f" % (joints[1] * R2D)))
        self.ui.rdoutElbowJC.setText(str("%+.2f" % (joints[2] * R2D)))
        self.ui.rdoutWristJC.setText(str("%+.2f" % (joints[3] * R2D)))

        if (len(joints) > 5):
            self.ui.rdoutWrist3JC.setText(str("%+.2f" % (joints[5] * R2D)))

        else:
            self.ui.rdoutWrist3JC.setText(str("N.A."))

    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f" % (pos[2])))
        self.ui.rdoutT.setText(str("%+.2f" % (pos[3])))

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rexarm.estop = True
        self.sm.set_current_state("estop")

    def sliderChange(self):
        """
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))

        self.ui.rdoutWrist2.setText(str(self.ui.sldrWrist2.value()))
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))

        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        self.rexarm.set_torque_limits([self.ui.sldrMaxTorque.value() / 100.0] * self.rexarm.num_joints,
                                      update_now=False)
        self.rexarm.set_speeds_normalized_global(self.ui.sldrSpeed.value() / 100.0, update_now=False)
        joint_positions = np.array([self.ui.sldrBase.value() * D2R,
                                    self.ui.sldrShoulder.value() * D2R,
                                    self.ui.sldrElbow.value() * D2R,
                                    self.ui.sldrWrist.value() * D2R,
                                    self.ui.sldrGrip1.value() * D2R])
        self.rexarm.set_positions(joint_positions, update_now=False)

    def directControlChk(self, state):
        if state == Qt.Checked:
            self.sm.set_current_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            self.sm.set_current_state("idle")
            self.ui.SliderFrame.setEnabled(False)

    def trackMouse(self):
        """
        Mouse position presentation in GUI
        TODO: Display the rgb/hsv value
        """

        x = QWidget.mapFromGlobal(self, QCursor.pos()).x()
        y = QWidget.mapFromGlobal(self, QCursor.pos()).y()
        if ((x < MIN_X) or (x >= MAX_X) or (y < MIN_Y) or (y >= MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutRGB.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,-)" % (x, y))

            rgb = self.rgb_image[y, x]  # [r, g, b] of this pixel
            hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2HSV)
            hsv = hsv_image[y, x]
            self.ui.rdoutRGB.setText("({},{},{})".format(rgb[0], rgb[1], rgb[2]))
            self.ui.rdoutHSV.setText("({},{},{})".format(hsv[0], hsv[1], hsv[2]))

    def mousePressEvent(self, QMouseEvent):
        """
        Function used to record mouse click positions for calibration
        """

        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return


"""main function"""


def main():
    app = QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
