#include <bot_msgs/oled_message_t.h>
#include <common/timestamp.h>

#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <map>

#include <unistd.h>

#include "ros/ros.h"

int main(int argc, char ** argv)
{
	ros::init(arc, argv, "rplidar_driver");
    ros::NodeHandle rosConnection;

    if(!rosConnection.ok()){ return 1; }

	std::map<std::string, ros::Publisher> pubs;     // map of topic names to publishers

	oled_message_t message;

	std::stringstream ss;
	time_t t = time(0);
	struct tm * now = localtime(&t);

	ss << std::setfill('0') << std::setw(2);

	while(1){
		t = time(0);
		now = localtime(&t);

		ss << std::setfill('0') << std::setw(2) << (now->tm_hour) 
			    << ':' << (now->tm_min) << ':' << now->tm_sec;
		std::cout << ss.str() << '\n';

		message.utime = utime_now();
		message.line1 = ss.str();
		message.line2 = "I really really really really <3 robots!";

		pubs[OLED_CHAN].publish(message);

		ss.clear();
		ss.str(std::string());
		usleep(1000000);
	}
//	oled_message_t message;
}
