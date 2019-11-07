#include <mbot/mbot_channels.h>
#include <bot_msgs/robot_path_t.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <map>
#include <string>

#include "ros/ros.h"


int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    int numTimes = 4;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(numTimes * 4);
    
    pose_xyt_t nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n] = nextPose;
    }
    
    nextPose.x = 1.0f;
    nextPose.y = 1.0f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 1] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 1.0f;
    nextPose.theta = -M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 2] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 3] = nextPose;
    }
    
    // Return to original heading after completing all circuits
    // nextPose.theta = 0.0f;
    // path.path.push_back(nextPose);
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    std::map<std::string, ros::Publisher> pubs;     // map of topic names to publishers
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    pubs[CONTROLLER_PATH_CHANNEL].publish(path);
    sleep(1);

    return 0;
}
