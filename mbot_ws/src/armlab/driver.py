import numpy as np
from time import sleep
import math


# main driver for challenge
class driver():
    def __init__(self, fsm):
        self.fsm = fsm
        self.current_state = 'start'
        self.funcs = dict()
        self.funcs['start'] = self.start
        self.funcs["Locate supply and trash"] = self.locate_supply_trash
        self.funcs['rotate for tags'] = self.rotate_for_tags
        self.funcs['pickup block'] = self.pickup_block
        self.tag = None
        self.tag_pose = None
        # self.funcs["detecting tags"] = self.detect_tags
        # self.funcs['moving'] = self.moving
        # self.funcs['dropping'] = self.dropping

    def detect_april(self, tag):
        homo = np.array([[tag.pose_R[0][0], tag.pose_R[0][1], tag.pose_R[0][2], tag.pose_t[0]],
                        [tag.pose_R[1][0], tag.pose_R[1][1], tag.pose_R[1][2], tag.pose_t[1]],
                        [tag.pose_R[2][0], tag.pose_R[2][1], tag.pose_R[2][2], tag.pose_t[2]],
                        [0.0, 0.0, 0.0, 1.0]], dtype='float')
        tag_pose = np.matmul(homo, np.array([0, 0, 0.0125, 1]))
        arm_pose = np.matmul(self.fsm.extrinsic, tag_pose)
        arm_pose[1] += .01
        arm_pose[2] += .01
        arm_pose[0] += .03
        return arm_pose

    def moving(self):
        if self.fsm.current_state != 'idle':
            # FSM is doing things. I assume it is doing things I told it to
            return
        self.fsm.tp.set_initial_wp()
        assert self.fsm.tag_pose is not None
        if self.fsm.tp.set_final_wp(self.fsm.tag_pose) is not None:
            print("moving arm")
            self.fsm.set_current_state("moving_arm")
        else:
            print("should move mbot")
            self.fsm.set_current_state("moving_mbot")

    def dropping(self):
        if self.fsm.current_state != 'idle':
            # FSM is doing things. I assume it is doing things I told it to
            return
        if self.fsm.trash_bin_pose is None:
            print("did not find trash bin yet")
            self.moving()
            return
        assert self.trash_bin_pose is not None
        if self.has_block is True:
            print("need to drop block in trash bin")
            self.set_current_state("drop_block")
        else:
            print("finished dropping block")
            self.current_state("done")

    def operate_task(self):
        while self.current_state != 'done':
            # if self.fsm.has_block is True:
            #     self.current_state = 'dropping'
            self.funcs[self.current_state]()
        print("DONE")
        return
        count = 0
        for tag in self.fsm.tags:
            if tag.tag_id == 1:
                count += 1
        if count == 1:
            self.detect_april(self.fsm.tags[0])
        elif count >= 1:
            self.calculate_push(self.fsm.tags)
            self.fsm.set_current_state("push_long_block")

        return
        if len(self.fsm.tags) > 0:
            self.detect_april(self.fsm.tags[0])
        else:
            self.fsm.set_current_state("idle")
            pass
        self.fsm.set_current_state("moving_arm")

    def start(self):
        self.fsm.set_current_state('idle')
        if len(self.fsm.tags) > 0:
            print("starting to locate supply and trash")
            self.current_state = "Locate supply and trash"
        else:
            print("\rWaiting for tags...", )

    def locate_supply_trash(self):
        """ At the start of the challenge run, this will locate the supply (and hopefully) the challenge areas """
        assert len(self.fsm.tags) > 0 #if this isn't true, then we r rekt
        supply_bin_arm_frame = self.helper_return_tag_pose(include_ids = [7])
        assert supply_bin_arm_frame is not None
        self.fsm.supply_bin_pose = self.fsm.arm_to_slam_pose(supply_bin_arm_frame)
        assert self.fsm.supply_bin_pose is not None
        print("supply_bin_pose: ", self.fsm.supply_bin_pose)
        print("Looking for trash bin")
        # now, rotate counterclockwise until we detect more id 7 tags
        trash_bin_arm_pose = None
        # Tested in lab, it took 3 rotations to see the trash bin
        assert self.fsm.turn_epsilon == 0.5 # otherwise the 3 count below may not be true
        self.rotate_mbot_and_wait()
        self.rotate_mbot_and_wait()
        self.rotate_mbot_and_wait()
        trash_bin_arm_pose = self.helper_return_tag_pose(include_ids = [7], exclude_arm_poses=[supply_bin_arm_frame]) # skip over the supply bin tag we've already seen. Realistically, not an issue
        assert trash_bin_arm_pose is not None
        self.fsm.trash_bin_pose = self.fsm.arm_to_slam_pose(trash_bin_arm_pose)
        self.current_state = 'rotate for tags'

    def rotate_for_tags(self):
        print("rotating looking for a block")
        rads_turned = 0
        tag_pose = self.helper_return_tag_pose(exclude_ids=[7])
        while tag_pose is None and rads_turned < 2 * np.pi:
            rads_turned += self.rotate_mbot_and_wait()
            tag_pose = self.helper_return_tag_pose(exclude_ids=[7])
            print("radians turned: ", rads_turned)
        print("Tag pose: ", tag_pose)
        self.tag_pose = tag_pose
        if self.tag_pose is not None:
            self.current_state = 'pickup block'
        else:
            self.current_state = 'done' #TODO: move this to explore further
        return tag_pose

    def drop_off_block(self):
        self.fsm.set_current_state('drop_block')
        while self.fsm.current_state != 'idle':
            pass # wait for state machine to drop block

    def go_to_area(self, area):
        """ ensure we have a block. Go to the trash bin, then drop off the block in the bin """
        assert self.have_block
        assert self.fsm.trash_bin_pose is not None
        assert area in ('trash', 'supply')
        # TODO: Do something smart about this
        # Since we start off facing the supply area, the X direction in SLAM frame is towards/away from
        # both the supply area and the garbage bin
        # so, go to (bin.x - 0.1, bin.y)
        area_pose = self.fsm.trash_bin_pose if area == 'trash' else self.fsm.supply_bin_pose
        TRASH_BIN_OFFSET = 0.1
        goal = (area_pose[0] - TRASH_BIN_OFFSET, area_pose[1], area_pose[2])
        goal[-1] = np.arctan2(goal[1] - self.fsm.slam_pose[1], goal[0] - self.fsm.slam_pose[0]) + self.fsm.slam_pose[-1] # update theta of goal
        self.fsm.set_current_state('moving_mbot', goal=goal)
        while self.fsm.current_state != 'idle':
            pass  # wait until we have stopped moving
        self.current_state = 'drop off block'

    def pickup_block(self):
        """ ensure there is a tag associated with a block.
            Detect its color and pick it up. If it is not reachable,
            Drive closer (a la task2.py). """
        assert self.tag is not None
        self.picking_up_block = False
        if self.fsm.current_state != 'idle':
            # I assume it is doing things I told it to
            return
        if self.picking_up_block:
            self.have_block = True
            # we have finished picking up the block. Assume it "just works"
            if self.fsm.block_type == 'trash_block':
                print("have trash block")
                # go to trash bin
                self.current_state = 'go to trash'
            else:
                print("have supply block")
                # go to supply area
                self.current_state = 'go to supply'
            return
        self.fsm.tp.set_initial_wp()
        self.fsm.tag_pose = self.tag_pose
        if self.fsm.tp.set_final_wp(self.fsm.tag_pose) is not None:
            self.picking_up_block = True
            print("moving arm")
            self.fsm.set_current_state("moving_arm")
        else:
            print("move mbot closer")
            self.fsm.set_current_state("moving_mbot")



    def rotate_mbot_and_wait(self):
        print("rotating mbot")
        original_theta = self.fsm.slam_pose[2]
        self.fsm.set_current_state('rotate_mbot')
        print("set state to rotate, waiting...")
        while self.fsm.current_state != 'idle':
            pass # wait for bot to finish rotating
        sleep(8) # wait for camera to stabilize
        angle_diff = self.fsm.slam_pose[2] - original_theta
        # TODO: make sure this conversion is correct.
        if angle_diff >= np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff <= -np.pi:
            angle_diff += 2 * np.pi
        return angle_diff

    def helper_return_tag_pose(self, exclude_ids=[], include_ids=None, exclude_arm_poses=[]):
        """ returns the pose of a tag relative to the arm base. Skips any ids in exclude_ids / tags with pose in exclude_arm_poses
        or ensures id is present in include_ids if include_ids is not None. If no exclude_ids or include_ids, it will return the pose of the first tag """
        if len(self.fsm.tags) == 0:
            return None
        for t in self.fsm.tags:
            if t.tag_id in exclude_ids:
                continue
            if include_ids is not None and t.tag_id not in include_ids:
                continue
            tag_pose = self.detect_april(t)
            if np.any([math.isclose(tag_pose, pose) for pose in exclude_arm_poses]): # TODO: ensure threshold for math.isclose is large enough
                continue
            self.tag = t
            return tag_pose
        return None

    def begin_task(self):
        """TODO"""
        pass
