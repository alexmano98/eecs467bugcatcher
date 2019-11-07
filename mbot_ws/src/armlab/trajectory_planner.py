import numpy as np 
import time
from copy import deepcopy

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan.
"""

def epsilonCompare(eps, val1, val2):
    return abs(val1 - val2) < eps

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints

        self.initial_wp = [0.0]*(self.num_joints - 1)
        self.final_wp = [0.0]*(self.num_joints - 1)

        self.isStopped = False # flag for stopping the robot

        self.go_order = [0, 3, 2, 1] # joint ordering for going to
        self.retract_order = [1, 2, 3, 0] # joint ordering for retracting

        self.move_increment = np.pi / 15 # 12 degree moves

    def set_initial_wp(self):
        '''
            Sets the initial waypoint to the current rexarm joint angles
        '''
        self.initial_wp = self.rexarm.joint_angles_fb.copy()

    def set_final_wp(self, waypoint, ik=True):
        '''
            Sets the final waypoint of the rexarm joint angles
            allows directly setting the final_wp
            Returns true if waypoint was set
        '''
        if ik:
            self.final_wp = self.rexarm.rexarm_IK(waypoint)
            if self.final_wp is None or any(np.isnan(self.final_wp)):
                return None
            self.final_wp = self.final_wp[0:4]
        else:
            self.final_wp = waypoint.copy()
        return True

    def swap_initial_final(self, oldgrip=False):
        '''
        changes initial and final, can keep or lose grip
        '''
        #self.initial_wp, self.final_wp = np.array(self.final_wp), np.array(self.initial_wp)
        temp = np.array(self.initial_wp).copy()
        self.initial_wp = np.array(self.final_wp).copy()
        if oldgrip:
            self.final_wp = temp
        else:
            self.final_wp = temp[0:-2] + [self.rexarm.joint_angles_fb[-1]]
        print("final wp ", self.final_wp)

    def go(self):
        '''
        allows the state machine to iterate through the path
        '''
        nextPosition = self.initial_wp.copy()
        for joint in self.go_order:
            nextPosition[joint] = self.final_wp[joint]
            yield nextPosition
            if self.isStopped:
                self.isStopped = False
                return None

    def customPath(self, ordering):
        '''
        allows a custom path
        '''
        nextPosition = self.initial_wp.copy()
        for joint in ordering:
            nextPosition[joint] = self.final_wp[joint]
            yield nextPosition
            if self.isStopped:
                self.isStopped = False
                return None

    def retract(self):
        '''
        retracting order
        '''
        nextPosition = self.initial_wp.copy()
        for joint in self.retract_order:
            nextPosition[joint] = self.final_wp[joint]
            yield nextPosition
            if self.isStopped:
                self.isStopped = False
                return None

    def stop(self):
        '''
        Stops the go generator
        '''

        self.isStopped = True
