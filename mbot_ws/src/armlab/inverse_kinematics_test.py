import unittest
import numpy as np
import math

from rexarm import Rexarm

class TestAll(unittest.TestCase):
    """
    This iterates over all but the last joint and uses FK to determine the position of the wrist joint. The gripper should be at (x, y, z - wrist_len). It then does the IK FK round trip to determine if IK is working
    """
    def setUp(self):
        num_joints = 5 # base, shoulder, elbow, wrist, grip
        self.rexarm = Rexarm(range(num_joints))

    def runner(self, wrist_to_endpoint_fn):
        angle_limits = self.rexarm.angle_limits
        for base_angle in np.arange(angle_limits[0][0], angle_limits[0][1], 0.0000001):
            for shoulder_angle in np.arange(angle_limits[1][0], angle_limits[1][1], 0.0000001):
                for elbow_angle in np.arange(angle_limits[2][0], angle_limits[2][1], 0.0000001):
                    # wrist angle has to be set in such a way that the tooltip points directly down.
                    # instead, compute where the wrist is, to determine where the tooltip is
                    self.rexarm.joint_angles_fb[0] = base_angle
                    self.rexarm.joint_angles_fb[1] = shoulder_angle
                    self.rexarm.joint_angles_fb[2] = elbow_angle
                    # wrist_angle is ignored in rexarm_FK(3)
                    elbow_pose = self.rexarm.rexarm_FK(3)
                    desired_pose = wrist_to_endpoint_fn(elbow_pose)
                    ik_angles = self.rexarm.rexarm_IK(desired_pose)
                    for i in range(self.rexarm.num_joints):
                        self.rexarm.joint_angles_fb[i] = ik_angles[i]
                    ik_pose = self.rexarm.rexarm_FK()
                    self.assertEqual(desired_pose, ik_pose)

    def testNormalReachSuccess(self):
        self.runner(lambda elbow_pose, rexarm: (elbow_pose[0], elbow_pose[1], elbow_pose[2] - rexarm.wrist_len, -90))

    def testHighReachSuccess(self):
        def convert(elbow_pose, rexarm):
            dx = rexarm.wrist_len * math.cos(rexarm.joint_angles_fb[0])
            dy = rexarm.wrist_len * math.sin(rexarm.joint_angles_fb[0])
            return (elbow_pose[0] + dx, elbow_pose[1] + dy, elbow_pose[2], 0)
        self.runner(convert)

    def testFarReachSuccess(self):
        # can't use self.runner for this, as that relies upon a fixed phi, but this phi can be variable
        angle_limits = self.rexarm.angle_limits
        for base_angle in np.arange(angle_limits[0][0], angle_limits[0][1], 0.0000001):
            for shoulder_angle in np.arange(angle_limits[1][0], angle_limits[1][1], 0.0000001):
                for elbow_angle in np.arange(angle_limits[2][0], angle_limits[2][1], 0.0000001):
                    for wrist_angle in np.arange(angle_limits[3][0], angle_limits[3][1], 0.0000001):
                        self.rexarm.joint_angles_fb[0] = base_angle
                        self.rexarm.joint_angles_fb[1] = shoulder_angle
                        self.rexarm.joint_angles_fb[2] = elbow_angle
                        self.rexarm.joint_angles_fb[3] = wrist_angle
                        desired_pose = self.rexarm.rexarm_FK()
                        ik_angles = self.rexarm.rexarm_IK(desired_pose)
                        for i in range(self.rexarm.num_joints):
                            self.rexarm.joint_angles_fb[i] = ik_angles[i]
                        ik_pose = self.rexarm.rexarm_FK()
                        self.assertEqual(desired_pose, ik_pose)


if __name__ == '__main__':
    unittest.main()
