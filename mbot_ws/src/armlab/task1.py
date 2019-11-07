import numpy as np


# sample structure for a complex task
class task1():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def detect_april(self, tag):
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                        [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                        [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0.0125, 1]))
        arm_pose = np.matmul(self.fsm.extrinsic, tag_pose)
        arm_pose[2] += .01
        arm_pose[1] += .01
        # shift pose of tag off center a bit so that the arm can pick it up without running into the block

        self.fsm.tag_pose = arm_pose
        print("tag pose ", self.fsm.tag_pose)

    def operate_task(self):
        if len(self.fsm.tags) > 0:
            self.detect_april(self.fsm.tags[0])
        else:
            self.fsm.set_current_state("idle")
            pass
        self.fsm.set_current_state("moving_arm")

    def begin_task(self):
        """TODO"""
        pass
