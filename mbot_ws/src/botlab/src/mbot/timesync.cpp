#include <bot_msgs/timestamp_t.h>
#include <common/time_util.h>
#include <mbot/mbot_channels.h>
#include <iostream>
#include <map>
#include <string>
#include "ros/ros.h"

/**
	A program that gets the current system time and publishes an ROS message of the current time
**/
int main(){

	//sleep duration between time samplings
	const int sleep_usec = 1000000;

	ros::init(arc, argv, "rplidar_driver");
    ros::NodeHandle rosConnection;

    if(!rosConnection.ok()){ return 1; }

    std::map<std::string, ros::Publisher> pubs;     // map of topic names to publishers

	timestamp_t now;

	while(true){

		now.utime = utime_now();

		pubs[MBOT_TIMESYNC_CHANNEL].publish(now);

		usleep(sleep_usec);
	}

	return 0;
}
