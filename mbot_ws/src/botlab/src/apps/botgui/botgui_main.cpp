#include <apps/botgui/botgui.hpp>
#include <thread>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "botgui");
    ros::NodeHandle nodeInstance;
    BotGui gui(&nodeInstance, argc, argv, 1000, 800, 15);
    ros::Rate loop_rate(10);
    
    std::thread rosThread([&]() {
        while(ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    });
    
    gui.run();
    
    return 0;
}
