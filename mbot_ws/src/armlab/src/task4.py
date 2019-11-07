import numpy as np
import time

# sample structure for a complex task
class task4():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_step = 0

    def detect_april(self, tag, num):
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                        [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                        [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0.0125, 1]))
        arm_pose = np.matmul(self.fsm.extrinsic, tag_pose)
        if num == 0:
            arm_pose[0] += .0185
            arm_pose[1] -= .0075
            arm_pose[2] -= .015
            arm_pose[-1] = -80
        else:
            arm_pose[-1] = -90
            arm_pose[0] += .04
            arm_pose[1] += .01
            arm_pose[2] -= .05
        #self.print_axes(np.matmul(self.fsm.extrinsic, homo))
        return arm_pose
    '''
    def print_axes(self, mat):
        
        Prints the tag axes wrt the world frame
        
        print("X-Axis: ", mat[0][0], " ", mat[1][0], " ", mat[2][0])
        
        print("Y-Axis: ", mat[0][1], " ", mat[1][1], " ", mat[2][1])

        print("Z-Axis: ", mat[0][2], " ", mat[1][2], " ", mat[2][2])


    def mag(self, vec):
        sum = 0
        for i in vec:
            sum = sum + i*i
        return np.sqrt(sum)
    
    def dot(self, vec1, vec2):
        sum = 0
        for i,j in zip(vec1, vec2):
            sum = sum + i*j
        return sum


    def shift_vec(self, axis, referenceFrame):
        
        returns a vector allowing world frame coordinates to be shifted by block frame coordinates
        
        block_axis = [0,0,0]
        if axis == 'x':
            block_axis = [referenceFrame[0][0], referenceFrame[1][0], referenceFrame[2][0], 0]
        elif axis == 'y':
            block_axis = [referenceFrame[0][1], referenceFrame[1][1], referenceFrame[2][1], 0]
        elif axis == 'z':
            block_axis = [referenceFrame[0][2], referenceFrame[1][2], referenceFrame[2][2], 0]
        return np.array(block_axis)

    def translate(self, vector, axis, scale, referenceFrame):
        
        Translate real position in blocks reference frame
        
        translation_vec = self.shift_vec(axis, referenceFrame)
        translated = np.add(vector, (scale * np.array(translation_vec)))
        return translated
    
    def scrape(self, tag1, tag2):
        armpose1 = self.detect_april(tag1)
        armpose2 = self.detect_april(tag2)

        self.fsm.tag_pose = armpose1

        a1_xy = armpose1[0:2]
        a2_xy = armpose2[0:2]

        y_axis = [0, 1]
        a1_theta = np.arccos( self.dot(a1_xy, y_axis) / ( self.mag(a1_xy) * self.mag(y_axis))) # distance from yaxis
        a2_theta = np.arccos( self.dot(a2_xy, y_axis) / ( self.mag(a2_xy) * self.mag(y_axis))) # dist froom y_axis

        theta = np.arccos( self.dot(a1_xy, a2_xy) / ( self.mag(a1_xy) * self.mag(a2_xy))) # angle btween vectors
        
        homo = np.array([[tag1.pose_R[0][0], tag1.pose_R[0][1], tag1.pose_R[0][2], tag1.pose_t[0]],
                        [tag1.pose_R[1][0], tag1.pose_R[1][1], tag1.pose_R[1][2], tag1.pose_t[1]],
                        [tag1.pose_R[2][0], tag1.pose_R[2][1], tag1.pose_R[2][2], tag1.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')

        self.fsm.zshift_vec = self.shift_vec('z', np.matmul(self.fsm.extrinsic, homo)) # z-axis in-out

        if a1_theta < a2_theta: # a1 right side tag
            #translate tag pose in tag frame
            self.fsm.yshift_vec = self.shift_vec('y', np.matmul(self.fsm.extrinsic, homo)) # (+) towards wall
            theta = -theta
        else:
            self.fsm.yshift_vec = -1 * self.shift_vec('y', np.matmul(self.fsm.extrinsic, homo)) # (-) away from wall

        # assumes cw is positive base rotation
        self.fsm.rotation = [theta, 0, 0, 0]
        # send arm to armpose1
        print("state machine is scraping")
        self.fsm.set_current_state("scrape_block")
    '''
    def scrape(self, tag):
        self.fsm.tag_pose = self.detect_april(tag, 0)
        self.fsm.set_current_state("scrape_block")

    def operate_task(self):
        if len(self.fsm.tags) < 1:
            return
        # assume only one block in corner
        self.scrape(self.fsm.tags[0])
        while self.fsm.current_state != 'idle' or len(self.fsm.tags) < 1:
            continue
        time.sleep(5)
        self.fsm.tag_pose = self.detect_april(self.fsm.tags[0], 1)
        self.fsm.set_current_state("moving_arm")
        
    def begin_task(self):
        """TODO"""
        pass
