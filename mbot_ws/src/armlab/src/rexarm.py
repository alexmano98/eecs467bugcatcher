# rexarm.py
import numpy as np
import time
import math

""" 
TODO:

Implement the missing functions
add anything you see fit

"""

""" Radians to/from  Degrees conversions """
D2R = 3.141592 / 180.0
R2D = 180.0 / 3.141592


def clamp_radians(theta):
    return np.clip(theta, -np.pi, np.pi)


class Rexarm():
    def __init__(self, joints):
        self.joints = joints
        self.gripper = joints[-1]
        self.gripper_open_pos = np.deg2rad(-60.0)
        self.gripper_closed_pos = np.deg2rad(2.0)   # TODO: setting how tight to squeeze the block
        self.gripper_state = True
        self.estop = False
        """TODO: Find the physical angle limits of the Rexarm. Remember to keep track of this if you include more motors"""
        self.angle_limits = np.array([
            [-145.00, -105.00, -120.00, -125.00, -80.00],
            [145.00, 105.00, 120.00, 125.00, 20.00]], dtype=np.float) * D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints  # degrees
        self.speed = [1.0] * self.num_joints  # 0 to 1
        self.max_torque = [1.0] * self.num_joints  # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints  # degrees
        self.speed_fb = [0.0] * self.num_joints  # 0 to 1
        self.load_fb = [0.0] * self.num_joints  # -1 to 1
        self.temp_fb = [0.0] * self.num_joints  # Celsius
        self.move_fb = [0] * self.num_joints

        """ Arm Lengths """
        # TODO: Fill in the measured dimensions.
        self.base_len = 5.0 / 100  # meters
        self.shoulder_len = 5.5 / 100  # ""
        self.elbow_len = 5.5 / 100  # ""
        self.wrist_len = 13.0 / 100  # ""

        """ DH Table """
        # TODO: Fill in the variables.
        self.dh_table = [{"theta": 0, "d": self.base_len, "a": 0, "alpha": np.pi / 2}, \
                         {"theta": np.pi / 2, "d": 0, "a": self.shoulder_len, "alpha": 0}, \
                         {"theta": 0, "d": 0, "a": self.elbow_len, "alpha": 0}, \
                         {"theta": 0, "d": 0, "a": self.wrist_len, "alpha": 0}]

        """ Issac added this """
        self.origin = np.array([0, 0, 0, 1])

    def initialize(self):
        for joint in self.joints:
            joint.enable_torque()
            joint.set_position(0.0)
            joint.set_torque_limit(0.5)    # TODO: higher torque limit, was 0.25
            joint.set_speed(0.25)

    def open_gripper(self):
        """ TODO """
        print("opening gripper")
        self.gripper.set_position(self.gripper_open_pos)
        
    def close_gripper(self):
        """ TODO """
        success = False
        gripper_set = self.gripper_closed_pos
        while not success:
            try:
                self.gripper.set_position(gripper_set)
                success = True
                time.sleep(0.5)
                break
            except:
                # gripper_set -= np.deg2rad(1.0)
                print("failed to set gripper position")

    def set_positions(self, joint_angles, update_now=True):
        joint_angles = self.clamp(joint_angles)
        print("Set position: ", joint_angles)
        for i, joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if (update_now):
                success = False
                while not success:
                    try:
                        joint.set_position(joint_angles[i])
                        success = True
                        time.sleep(0.1)
                        break
                    except:
                        pass  # never quit
                    
    def set_speeds_normalized_global(self, speed, update_now=True):
        for i, joint in enumerate(self.joints):
            self.speed[i] = speed
            if (update_now):
                success = False
                while not success:
                    try:
                        joint.set_speed(speed)
                        success = True
                        time.sleep(0.1)
                        break
                    except:
                        pass # never quit

    def set_speeds_normalized(self, speeds, update_now=True):
        for i, joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if (update_now):
                success = False
                while not success:
                    try:
                        joint.set_speed(speeds[i])
                        success = True
                        time.sleep(0.1)
                        break
                    except:
                        pass  # never quit

    def set_speeds(self, speeds, update_now=True):
        for i, joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i] / joint.max_speed)
            if (speed_msg < 3.0 / 1023.0):
                speed_msg = 3.0 / 1023.0
            if (update_now):
                success = False
                while not success:
                    try:
                        joint.set_speed(speed_msg)
                        success = True
                        time.sleep(0.1)
                        break
                    except:
                        pass  # never quit

    def set_torque_limits(self, torques, update_now=True):
        for i, joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if (update_now):
                success = False
                while not success:
                    try:
                        joint.set_torque_limit(torques[i])
                        success = True
                        time.sleep(0.1)
                        break
                    except:
                        pass  # never quit

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        for i, joint in enumerate(self.joints):
            self.joint_angles_fb[i] = joint.get_position()
        return self.joint_angles_fb

    def get_speeds(self):
        for i, joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed()
        return self.speed_fb

    def get_loads(self):
        for i, joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load()
        return self.load_fb

    def get_temps(self):
        for i, joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp()
        return self.temp_fb

    def get_moving_status(self):
        for i, joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving()
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while ((time.time() - time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if (self.estop == True):
                break

    def clamp(self, joint_angles):
        """TODO: Implement this function to clamp the joint angles"""
        for i, angle in enumerate(joint_angles):
            joint_angles[i] = np.clip(angle, self.angle_limits[0][i], self.angle_limits[1][i])
        return joint_angles

    def calc_A_FK(self, theta, link):
        """
        TODO: Implement this function
        theta is radians of the link number
        link is the index of the joint, and it is 0 indexed (0 is base, 1 is shoulder ...)
        returns a matrix A(2D array)
        """
        dh_params = self.dh_table[link]
        rot_x_alpha = np.array([
            [1, 0, 0, 0],
            [0, math.cos(dh_params["alpha"]), -math.sin(dh_params["alpha"]), 0],
            [0, math.sin(dh_params["alpha"]), math.cos(dh_params["alpha"]), 0],
            [0, 0, 0, 1]
        ])

        trans_x_a = np.array([
            [1, 0, 0, dh_params["a"]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        trans_z_d = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, dh_params["d"]],
            [0, 0, 0, 1]
        ])

        rot_z_theta = np.array([
            [math.cos(dh_params["theta"] + theta), -math.sin(dh_params["theta"] + theta), 0, 0],
            [math.sin(dh_params["theta"] + theta), math.cos(dh_params["theta"] + theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return np.matmul(np.matmul(rot_z_theta, trans_z_d), np.matmul(trans_x_a, rot_x_alpha))

    def rexarm_FK(self, joint_num=4):
        """
        TODO: implement this function

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the
        desired link
        """
        A = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        for joint in range(joint_num):
            A = np.matmul(A, self.calc_A_FK(self.joint_angles_fb[joint], joint))

        return np.matmul(A, self.origin)

    def rexarm_IK(self, pose):
        """
        TODO: implement this function

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.
        If the gripper is perpendicular to the floor and facing down,
        then phi is -90 degree.
        If the gripper is parallel to the floor,
        then phi is 0 degree.
        returns a 4-tuple of joint angles or None if configuration is impossible
        """
        x, y, z, phi = pose
        print("Pose ", pose)
        theta1 = np.arctan2(y, x)

        # normal reach
        if phi == -90:
            R = np.sqrt(x ** 2 + y ** 2)
            M = np.sqrt(R ** 2 + (self.wrist_len + z - self.base_len) ** 2)
            print("M ", M)
            alpha = np.arctan2(self.wrist_len + z - self.base_len, R)
            beta = np.arccos((-(self.elbow_len ** 2) + self.shoulder_len ** 2 + M ** 2) / (2 * self.shoulder_len * M))
            gamma = np.arccos(
                (-(M ** 2) + self.shoulder_len ** 2 + self.elbow_len ** 2) / (2 * self.shoulder_len * self.elbow_len))
            theta2 = -((np.pi / 2) - alpha - beta)
            theta3 = -(np.pi - gamma)
            theta4 = -(np.pi - theta2 - theta3)
        # high reach
        elif phi == 0:
            R = z - self.base_len
            M = np.sqrt(R ** 2 + (self.wrist_len - np.sqrt(x ** 2 + y ** 2)) ** 2)

            # M = np.sqrt( R**2 + (self.wrist_len - x)**2 )
            beta_2 = np.arccos(R / M)
            alpha_2 = np.arcsin(R / M)

            beta_1 = np.arccos((-(self.elbow_len ** 2) + (self.shoulder_len ** 2) + (M ** 2))
                               / (2 * self.shoulder_len * M))
            gamma = np.arccos((-(M ** 2) + (self.elbow_len ** 2) + (self.shoulder_len ** 2))
                              / (2 * self.shoulder_len * self.elbow_len))
            alpha_1 = np.pi - gamma - beta_1

            alpha = alpha_1 + alpha_2
            beta = beta_1 + beta_2

            theta2 = -beta
            theta3 = -(np.pi - gamma)
            theta4 = -(np.pi - alpha)

            '''
            alpha = np.arctan2(self.wrist_len - np.sqrt(x ** 2 + y ** 2), R)
            beta = np.arccos((-(self.elbow_len ** 2) + self.shoulder_len ** 2 + M ** 2) / (2 * self.shoulder_len * M))
            gamma = np.arccos(
                (-(M ** 2) + self.shoulder_len ** 2 + self.elbow_len ** 2) / (2 * self.shoulder_len * self.elbow_len))
            theta2 = -alpha - beta
            theta3 = np.pi - gamma
            theta4 = (np.pi / 2) - theta2 - theta3
            if z > (self.base_len + self.shoulder_len + self.elbow_len) or np.sqrt(x ** 2 + y ** 2) > self.wrist_len:
                return None
            '''
        # far reach
        else:
            '''
            R = np.sqrt(x ** 2 + y ** 2)
            M = sqrt(R**2 + self.base_len ** 2)

            alpha = np.arcsin( M / R )

            beta = np.arccos( ( -(self.wrist_len ** 2) + (self.shoulder_len + self.elbow_len)**2 + M**2) 
                                / (2 * (self.shoulder_len + self.elbow_len) * M)
            gamma = np.arccos( (-(M ** 2) + (self.shoulder_len + self.elbow_len) ** 2 + self.wrist_len ** 2) 
                                / (2 * (self.shoulder_len + self.elbow_len) * self.wrist_len) )
            theta2 = np.pi - beta - alpha
            theta3 = 0
            theta4 = np.pi - gamma
            suggestions
            '''
            R = np.sqrt(x ** 2 + y ** 2)
            theta2 = -np.pi / 2
            theta3 = 0
            theta4 = -(np.arctan2(self.base_len - z, R - self.shoulder_len - self.elbow_len))
            if np.sqrt((self.base_len - z) ** 2 + (R - self.shoulder_len - self.elbow_len) ** 2) > self.wrist_len:
                return None

        if theta1 < self.angle_limits[0][0] or theta1 > self.angle_limits[1][0]:
            return None
        if theta2 < self.angle_limits[0][1] or theta2 > self.angle_limits[1][1]:
            return None
        if theta3 < self.angle_limits[0][2] or theta3 > self.angle_limits[1][2]:
            return None
        if theta4 < self.angle_limits[0][3] or theta4 > self.angle_limits[1][3]:
            return None
        return theta1, theta2, theta3, theta4
