int64 utime

int8 STATE_STOPPED = 0
int8 STATE_MOVING = 1
int8 STATE_ROTATE = 2
int8 state

pose_xyt_t goal_pose

int32 num_obstacles

pose_xyt_t[num_obstacles] obstacle_poses[num_obstacles]

double theta