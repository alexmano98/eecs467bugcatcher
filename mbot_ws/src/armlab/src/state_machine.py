import time
import numpy as np
import cv2
import math

import rospy

import os

# Messages used for communication with Mbot programs.
from bot_msgs.msg import pose_xyt_t
from bot_msgs.msg import occupancy_grid_t
from bot_msgs.msg import mbot_status_t
from bot_msgs.msg import mbot_command_t

D2R = 3.141592/180.0
R2D = 180.0/3.141592

eps = 0.15
TURN_EPSILON = 0.5

"""
TODO: Add states and state functions to this class
        to implement all of the required logics
"""
class StateMachine():
    def __init__(self, rexarm, planner):
        self.rexarm = rexarm
        self.tp = planner
        self.tags = []
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.block_type = 'none'
        self.original_pose = [0.0] * 4
        self.original_tag = [0.0] * 4
        
        self.isHome = False

        self.rotation = [0.0] * 4
        self.xshift_vec = [0.0] * 4 #shift the xyz coordinates wrt block frame
        self.yshift_vec = [0.0] * 4 
        self.zshift_vec = [0.0] * 4

        self.home = [0.0, 0.0, np.deg2rad(-90.0), 0.0]

        self.lc = lcm.LCM()
        lcmSLAMPoseSub = self.lc.subscribe("SLAM_POSE", self.slampose_feedback_handler)
        lcmSLAMPoseSub.set_queue_capacity(1)

        lcmMbotStatusSub = self.lc.subscribe("MBOT_STATUS", self.mbotstatus_feedback_handler)

        # TODO: Add more variables here, such as RBG/HSV image here.
        self.current_step = 0
        self.path = []
        self.start_time = 0
        self.duration = 0
        self.slam_pose = None
        self.mbot_goal_pose = None
        self.mbot_status = None
        self.mbot_finished = False
        self.turn_epsilon = TURN_EPSILON

        self.tag_pose = None  # the pose of block from april tag

        self.extrinsic = np.array([[-0.02963398, -0.30054063, 0.95330853, 0.01931047],
                                   [-0.99810848, 0.06029093, -0.01201925, 0.00316656],
                                   [-0.05386358, -0.95186151, -0.30175881, 0.01349531],
                                   [0., 0., 0., 1.]])
        '''
        camera_rotation = 60  # in degrees
        camera_translation = [3.5 / 100, 0, 0]  # camera trans in m
        self.extrinsic = np.array([
            [np.cos(camera_rotation), 0, -np.sin(camera_rotation), camera_translation[0]],
            [0, 1, 0, camera_translation[1]],
            [np.sin(camera_rotation), 0, np.cos(camera_rotation), camera_translation[2]],
            [0, 0, 0, 1],
        ])
        '''
        # extrinsic homogeneous transformation matrix

    def set_current_state(self, state):
        self.mbot_goal_pose = None
        self.mbot_finished = False
        self.mbot_status = None
        self.current_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if self.current_state == "manual":
            self.manual()

        if self.current_state == "idle":
            self.idle()
                
        if self.current_state == "estop":
            self.estop()  

        if self.current_state == "calibrate":
            self.calibrate()

        if self.current_state == "moving_arm":
            self.moving_arm()

        if self.current_state == "moving_mbot":
            self.moving_mbot()

        if self.current_state == "push_long_block":
            self.push_long_block()

        if self.current_state == "scrape_block":
            self.scrape_block()

        if self.current_state == "drop_block":
            self.drop_block()

        if self.current_state == "rotate_mbot":
            self.rotate_mbot()

        self.get_mbot_feedback()
        self.rexarm.get_feedback()
               

    """Functions run for each state"""
    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.rexarm.send_commands()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        if not self.isHome:
            self.arm_home()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.rexarm.disable_torque()
        
    def calibrate(self):
        """Perform camera calibration here
        TODO: Use appropriate cameraMatrix (intrinsics) parameters
        Change the arm frame 3d coordinate based on the point that you choose.
        Store the extrinsic matrix and load it on start.
        """
        cameraMatrix = np.array([[604.37968515, 0, 325.93761715], [0, 603.92764597, 241.48766872], [0, 0, 1]])
        # 3D coordinates of the center of AprilTags in the arm frame in meters.
        objectPoints = np.array([[0.15, 0.0254/2, -.057 + 0.0254/2],
                                 [0.15 + 0.0254, -0.0254/2, -.057 + 0.0254/2],
                                 [0.15, 0.0254/2, -.057 + (3 * 0.0254/2)],
                                 [0.15 + 0.0254, -0.0254/2, -.057 + (3 * .0254/2)]])

        # Use the center of the tags as image points. Make sure they correspond to the 3D points.
        imagePoints = np.array([tag.center for tag in self.tags])
        assert(len(self.tags) == 4)
        print("Image Points: ", imagePoints)
        success, rvec, tvec = cv2.solvePnP(objectPoints, np.array(imagePoints), cameraMatrix, None)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        # print("rotation matrix ", rotation_matrix)
        # print("tvec ", tvec)

        # TODO: implement the function that transform pose_t of the tag to the arm's
        # frame of reference.
        # homogeneous matrix from arm coordinates to camera coordinates
        homo_a2c = np.array([[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], tvec[0]],
                            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], tvec[1]],
                            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], tvec[2]],
                            [0.0, 0.0, 0.0, 1.0]], dtype='float')
        # homogeneous matrix from camera coordinates to robot arm coordinates
        self.extrinsic = np.linalg.inv(homo_a2c)
        print("extrinsic: ", self.extrinsic)

        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

        # Set the next state to be idle
        self.set_current_state("idle")

    def arm_goto(self, pose, ordering=None):
        ''' Sends the arm to a position '''
        print("moving arm to position")
        self.tp.set_final_wp(pose)
        if ordering is not None:
            for joint in self.tp.customPath(ordering):
                for i, v in enumerate(joint):
                    self.rexarm.position[i] = v
                self.rexarm.send_commands()
                time.sleep(.5)
        else:
            for joint in self.tp.go():
                for i, v in enumerate(joint):
                    self.rexarm.position[i] = v
                self.rexarm.send_commands()
                time.sleep(0.5)
        time.sleep(1)
    
    def arm_rotate(self):
        '''
        slowly rotates to position and clears the rotation variable
        '''
        print("rotating arm")
        self.rexarm.set_speeds_normalized_global(.1)
        self.rexarm.send_commands()
        time.sleep(.1)

        for i, theta in enumerate(self.rotation):
            if theta != 0:
                self.rexarm.position[i] = self.rexarm.position[i] + theta
                self.rexarm.send_commands()
                time.sleep(.5)
        self.rotation = [0.0] * 4
        
    def arm_home(self):
        '''
        sends the arm to the home position and enables home flag
        '''
        print("going home")
        self.rexarm.set_speeds_normalized_global(.1)
        self.rexarm.send_commands()
        time.sleep(.1)

        for i, joint in enumerate(self.home):
            self.rexarm.position[i] = joint
        self.rexarm.send_commands()
        time.sleep(.5)
        self.isHome = True

    def arm_out(self):
        ''' Sends the arm to a position '''
        print("moving arm to position")
        self.tp.set_final_wp(self.home, False)
        for joint in self.tp.customPath([1,2,3, 0]):
            for i, v in enumerate(joint):
                self.rexarm.position[i] = v
            self.rexarm.send_commands()
            time.sleep(0.5)
        time.sleep(1)

    def moving_arm(self):
        self.rexarm.set_speeds_normalized_global(.1)

        time.sleep(1)

        block_pose = self.tag_pose
        self.tp.set_initial_wp()
        assert self.tp.set_final_wp(block_pose)
        self.rexarm.open_gripper()
        time.sleep(0.5)
        for joint in self.tp.go():
            print("\rjoint: ", joint)
            joint[-1] = np.deg2rad(-60.0)
            for i, v in enumerate(joint):
                self.rexarm.position[i] = v
            self.rexarm.send_commands()
            time.sleep(0.5)
        self.rexarm.close_gripper()
        time.sleep(1)

        self.tp.set_final_wp([0, 0, np.deg2rad(-90), 0], False)
        for joint in self.tp.retract():
            for i, v in enumerate(joint):
                self.rexarm.position[i] = v
            self.rexarm.send_commands()
            time.sleep(0.5)

        print("done")
        self.set_current_state("idle")

    def arm_to_slam_pose(self, arm_pose, esp=0.0):
        """ goes from arm_pose to slam pose. Will adjust by espsilon in the -x direction (closer to the bot) """
        x, y, theta = self.slam_pose[0], self.slam_pose[1], self.slam_pose[2]
        arm_to_slam_trasform = np.array([[math.cos(theta), -math.sin(theta), x - eps],
                              [math.sin(theta), math.cos(theta), y],
                              [0, 0, 1]
                            ])
        goal = np.matmul(arm_to_slam_trasform, np.array([arm_pose[0], arm_pose[1], 1]))
        # need to overwrite theta to have us face the real goal (if esp is non-0. If it is, then we will look in the direction we headed to get to the goal)
        # TODO: have this be just a rotation transform
        goal[-1] = np.arctan2(goal[1] - y, goal[0] - x) + theta
        return goal

    def moving_mbot(self, goal=None):
        """ Will move the mbot to esp away from a block, then go idle """
        if self.mbot_goal_pose is not None:
            if self.mbot_status == mbot_status_t.STATUS_COMPLETE or self.mbot_finished:
                print("reached goal")
                self.set_current_state('idle')
            else:
                print("Mbot progress: ", self.mbot_status)
            return
        if goal is None:
            goal = self.arm_to_slam_pose(self.tag_pose, esp=eps)
        # TODO: tune the eps for how close we want to drive to the block
        print("driving the mbot to ", goal)
        self.publish_mbot_command(mbot_command_t.STATE_MOVING, goal)
        self.mbot_goal_pose = goal

    def slampose_feedback_handler(self, channel, data):
        """
        Feedback Handler for slam pose
        this is run when a feedback message is recieved
        """
        msg = pose_xyt_t.decode(data)
        self.slam_pose = (msg.x, msg.y, msg.theta)

    def mbotstatus_feedback_handler(self, channel, data):
        """
        Feedback Handler for mbot status
        this is run when a feedback message is recieved
        """
        msg = mbot_status_t.decode(data)
        self.mbot_status = msg.status
        self.mbot_finished = (self.mbot_status == mbot_status_t.STATUS_COMPLETE)
        if self.mbot_finished:
            self.current_state = 'idle'
            print("I KNOW THE MBOT HAS FINISHED MOVING")

    def get_mbot_feedback(self):
        """
        LCM Handler function
        Must be called continuously in the loop to get feedback.
        """

        self.lc.handle_timeout(10)

    def publish_mbot_command(self, state, goal_pose, obstacles=[], theta=TURN_EPSILON):
        """
        Publishes mbot command.
        """
        msg = mbot_command_t()
        msg.utime = int(time.time() * 1e6)
        msg.state = state

        if state == mbot_command_t.STATE_STOPPED:
            pass
        elif state == mbot_command_t.STATE_MOVING:
            msg.goal_pose.x, msg.goal_pose.y, msg.goal_pose.theta = goal_pose[0], goal_pose[1], goal_pose[2]
            msg.num_obstacles = len(obstacles)
            for i in range(msg.num_obstacles):
                obs_pose = pose_xyt_t()
                obs_pose.utime = int(time.time() * 1e6)
                obs_pose.x, obs_pose.y, obs_pose.theta = obstacles[i]
                msg.obstacle_poses.append(obs_pose)
        elif state == mbot_command_t.STATE_ROTATE:
            msg.theta = theta
        else:
            raise NameError('Unknown mbot commanded state')

        self.lc.publish("MBOT_COMMAND", msg.encode())

    """TODO: Add more functions and states in the state machine as needed"""

    def push_long_block(self):
        print('pushing long block')
        theta1 = np.arctan2(self.tag_pose[1], self.tag_pose[0])
        self.rexarm.set_speeds_normalized_global(.1)
        self.rexarm.set_positions([theta1,0,0,0,np.deg2rad(20)])
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[3] = np.deg2rad(-90)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[2] = np.deg2rad(-90)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[1] = np.deg2rad(-10)
        self.rexarm.send_commands()
        time.sleep(.5)
        for x in range(9):
            self.rexarm.position[2] += np.deg2rad(10)
            self.rexarm.position[3] += np.deg2rad(10)
            self.rexarm.position[1] -= np.deg2rad(10)
            self.rexarm.send_commands()
            time.sleep(.5)
        self.rexarm.set_positions([0,0,0,0,0])
        self.rexarm.send_commands()
        print("done")
        self.set_current_state("idle")

    def scrape_block(self):
        print("Scraping block from corner")
        '''if self.tag_pose is None:
            self.set_current_state("idle")
            return

        wall_edge = (2.5 / 100) / 4
        backup_dist = (-5 / 100)  # back up half a cm

        base_rotation = self.rotation[0]

        # shut the gripper
        self.rexarm.position[-1] = 70
        self.rexarm.send_commands()
        time.sleep(.5)

        tag_pose = self.tag_pose

        # shifts the target position based on the block frame
        edge_away_from_wall = np.add(np.add(np.array(self.tag_pose), wall_edge * np.array(self.xshift_vec)),
                                     backup_dist * np.array(self.zshift_vec))

        edge_away_from_wall[2] = (2.5 / 100 / 2)  # middle of block
        print("first pose: ", edge_away_from_wall)
        edge_away_from_wall[2] = (2.5 / 100 / 2)  # middle of block
        self.arm_goto(edge_away_from_wall, [0, 3, 2, 1])
        edge_away_from_wall[2] = (2.5 / 100 / 2)  # middle of block
        self.arm_rotate()
        self.arm_out()

        edge_towards_wall = np.add(np.add(np.array(self.tag_pose), -wall_edge * np.array(self.xshift_vec)),
                                   backup_dist * np.array(self.zshift_vec))

        print("second pose: ", edge_towards_wall)
        edge_towards_wall[2] = (2.5 / 100 / 2)  # middle of block
        print('second pose: reprint ', edge_towards_wall)
        self.arm_goto(edge_towards_wall, [0, 3, 2, 1])
        self.rotation[0] = base_rotation
        self.arm_rotate()
        self.arm_out()
        '''
        angles = self.rexarm.rexarm_IK(self.tag_pose)
        print(angles)
        self.rexarm.position[0] = angles[0]
        self.rexarm.position[4] = np.deg2rad(20)
        self.rexarm.position[3] = angles[3] - np.deg2rad(20)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[2] = angles[2]
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[1] = angles[1] + np.deg2rad(20)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[3] = angles[3] + np.deg2rad(20)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[1] = angles[1] - np.deg2rad(10)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.set_speeds_normalized_global(1)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[3] = angles[3] - np.deg2rad(30)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.position[0] = np.deg2rad(90)
        self.rexarm.send_commands()
        time.sleep(.5)
        self.rexarm.set_positions([0,0,0,0,0])
        self.rexarm.send_commands()
        self.set_current_state("idle")

    def drop_block(self):
        self.tp.set_final_wp(self.home)
        for joint in self.tp.go():
            for i, v in enumerate(joint):
                self.rexarm.position[i] = v
            self.rexarm.send_commands()
            time.sleep(0.5)
        self.rexarm.open_gripper()
        time.sleep(0.5)

        self.tp.set_final_wp([0, 0, 0, 0], False)
        for joint in self.tp.retract():
            for i, v in enumerate(joint):
                self.rexarm.position[i] = v
            self.rexarm.send_commands()
            time.sleep(0.5)
        self.set_current_state('idle')

    def rotate_mbot(self):
        """ Should rotate the mbot counterclockwise by a small amount then go back idle """
        if self.mbot_goal_pose is not None:
            if self.mbot_status == mbot_status_t.STATUS_COMPLETE or self.mbot_finished:
                print(self.mbot_status == mbot_status_t.STATUS_COMPLETE)
                print(self.mbot_finished)
                print("reached goal")
                self.set_current_state('idle')
            else:
                print("Assuming it is still going")
                print("mbot status: ", self.mbot_status)
            return
        target_theta = self.slam_pose[2] + self.turn_epsilon
        if target_theta >= np.pi:
            target_theta -= 2 * np.pi
        elif target_theta <= -np.pi:
            target_theta += 2 * np.pi
        self.mbot_goal_pose = (self.slam_pose[0], self.slam_pose[1], target_theta)
        print("Going to: ", self.mbot_goal_pose)
        self.publish_mbot_command(mbot_command_t.STATE_ROTATE, self.mbot_goal_pose)

    def throw_block_at_tony(self):
        pass

    def pick_up_jeremy(self):
        pass
