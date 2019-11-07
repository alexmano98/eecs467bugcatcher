import numpy as np


# sample structure for a complex task
class task3():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def detect_april(self, tag):
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                        [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                        [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0.025, 1]))
        arm_pose = np.matmul(self.fsm.extrinsic, tag_pose)
        arm_pose[2] += .01
        arm_pose[1] += .025
        self.fsm.tag_pose = arm_pose

    def calculate_push(self, tags):
        #calculate poses
        poses = []
        for tag in tags:
            if tag.tag_id != 1:
                continue
            homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]], [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]], [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]], [0.0, 0.0, 0.0, 1.0]], dtype='float')
            tag_pose = np.matmul(homo, np.array([0, 0, 0, 1]))
            poses.append(np.matmul(self.fsm.extrinsic, tag_pose))
        furthest = poses[0]
        dist = poses[0][0]
        
        #find furthest x value, easiest tag to push
        for pose in poses:
            if pose[0] > dist:
                furthest = pose
                dist = pose[0]
        for pose in poses:
            if abs(dist - pose[0]) < .02 and furthest[1] != pose[1]:
                if furthest[1] < pose[1]:
                    furthest[1] -= .01
                else:
                    furthest[1] += .01
                break
        self.fsm.tag_pose = furthest
        
    def operate_task(self):
        count = 0
        for tag in self.fsm.tags:
            if tag.tag_id == 1:
                count += 1
        if count == 1:
            self.detect_april(self.fsm.tags[0])
        elif count >= 1:
            self.calculate_push(self.fsm.tags)
            self.fsm.set_current_state("push_long_block")
        while self.fsm.current_state != 'idle' or len(self.fsm.tags) == 0:
            continue
        if len(self.fsm.tags) > 0:
            self.detect_april(self.fsm.tags[0])
        self.fsm.set_current_state("moving_arm")
        return
        self.fsm.set_current_state("moving_arm")

    def begin_task(self):
        """TODO"""
        pass
