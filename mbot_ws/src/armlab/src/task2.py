import numpy as np


# sample structure for a complex task
class task2():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_state = 'start'
        self.funcs = dict()
        self.funcs['start'] = self.start
        self.funcs["detecting tags"] = self.detect_tags
        self.funcs['moving'] = self.moving

    def detect_april(self, tag):
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                        [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                        [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0.015, 1]))
        arm_pose = np.matmul(self.fsm.extrinsic, tag_pose)
        arm_pose[1] += .01
        arm_pose[2] += .01
        arm_pose[0] += .03
        self.fsm.tag_pose = arm_pose

    def detect_tags(self):
        if len(self.fsm.tags) > 0:
            self.detect_april(self.fsm.tags[0])
            print("detected a tag I hope!")
            print("fsm tag_pose: ", self.fsm.tag_pose)
            self.current_state = 'moving'
            return
        print("No tags to detect!")
        self.current_state = 'start'

    def moving(self):
        if self.fsm.current_state != 'idle':
            # FSM is doing things. I assume it is doing things I told it to
            return
        if len(self.fsm.tags) > 0:
            self.detect_april(self.fsm.tags[0])
        #else: spin?
        self.fsm.tp.set_initial_wp()
        assert self.fsm.tag_pose is not None
        if self.fsm.tp.set_final_wp(self.fsm.tag_pose) is not None:
            print("should move arm")
            self.fsm.set_current_state("moving_arm")
        else:
            print("should move mbot")
            self.fsm.set_current_state("moving_mbot")

    def operate_task(self):
        while self.current_state != 'done':
            self.funcs[self.current_state]()

    def start(self):
        self.fsm.set_current_state('idle')
        if len(self.fsm.tags) > 0:
            self.current_state = "detecting tags"
        else:
            print("\rWaiting for tags...", )

    def begin_task(self):
        """TODO"""
        pass
