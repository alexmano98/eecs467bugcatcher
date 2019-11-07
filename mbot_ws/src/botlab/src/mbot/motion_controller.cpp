#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <bot_msgs/mbot_motor_command_t.h>
#include <bot_msgs/odometry_t.h>
#include <bot_msgs/pose_xyt_t.h>
#include <bot_msgs/robot_path_t.h>
#include <bot_msgs/timestamp_t.h>
#include <bot_msgs/message_received_t.h>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <slam/slam_channels.h>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include <vector>

#include "ros/ros.h"


int num_points_reached = 3; // number of points reached in the current target path


float clamp_speed(float speed)
{
    if(speed < -1.0f)
    {
        return -1.0f;
    }
    else if(speed > 1.0f)
    {
        return 1.0f;
    }
    
    return speed;
}


class MotionController
{
public:
    
    /**
    * Constructor for MotionController.
    */
    MotionController(ros::NodeHandle * nodeInstance) : nodeInstance_(nodeInstance)
    {
        ////////// TODO: Initialize your controller state //////////////
        
        // Initially, there's no offset between odometry and the global state
        odomToGlobalFrame_.x = 0.0f;
        odomToGlobalFrame_.y = 0.0f;
        odomToGlobalFrame_.theta = 0.0f;

	    time_offset = 0;
	    timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";
    }
    
    /**
    * updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        //////////// TODO: Implement your feedback controller here. //////////////////////
        
        const float kPGain = 1.0f;
        const float kDGain = 0.01f;
        const float kIGain = 0.3f;

        const float kPTurnGain = 1.0f; // was 1.5
        const float kDesiredSpeed = 0.15f;
        const float kMinSpeed = 0.12f;
        const float kTurnSpeed = 0.6f;
        const float kTurnMaxSpeed = 0.4f;
        const float slowDownDistance = 0.15f;
        
        mbot_motor_command_t cmd;

        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;

        cmd.utime = now();
        
        if(haveReachedTarget())
        {
		std::cout << "TARGET REACHED\n";
            bool haveTarget = assignNextTarget();
            
            if(!haveTarget)
            {
                std::cout << "COMPLETED PATH!\n";
            }
        }
        
        if(!targets_.empty() && !odomTrace_.empty())
        {
            // Use feedback based on heading error for line-of-sight vector pointing to the target.
            pose_xyt_t target = targets_.back();
            
            // Convert odometry to the global coordinates
            pose_xyt_t pose = currentPose();
            
            double targetHeading = std::atan2(target.y - pose.y, target.x - pose.x);
            double error = angle_diff(targetHeading, pose.theta);
            std::cout << "targetHeading: " << targetHeading << ", pose Theta: " << pose.theta << std::endl;
            std::cout << "Angle error:" << error << '\n';
            std::cout << "target: " << target.x << " " << target.y << " " << target.theta << std::endl;
            if(state_ == TURN)
            {
                if(std::abs(error) > 0.05) // turn in place until pointed approximately at the target
                {

                    cmd.trans_v = 0; //set translational velocity to 0

                    float turnspeed = 0.0;
                    if (error > 0) {
                        turnspeed = kTurnSpeed;
                    } else {
                        turnspeed = -kTurnSpeed;
                    }

                    if (std::abs(error) < 0.5) {
                        // kick in PID close to end
                        double deltaError = error - lastError_;
                        totalError_ += error;
                        lastError_ = error;

                        turnspeed = (error * kPGain) + (deltaError * kDGain) + (totalError_ * kIGain);
                        if (turnspeed >= 0) {
                            turnspeed = std::min(turnspeed, kTurnMaxSpeed);
                            //turnspeed = std::max(turnspeed, kMinSpeed / 2);
                        } else {
                            turnspeed = std::max(turnspeed, -kTurnMaxSpeed);
                            //turnspeed = std::min(-turnspeed, -kMinSpeed / 2);
                        }

                        //std::cout << "Turnspeed: " << turnspeed;
                        
                    }
                    // std::cout << "turnspeed: " << turnspeed << "\n";
                    // Turn left if the target is to the left
                    if(error > 0.0)
                    {
                        //std::cout << "Turning left\n";
                    }
                    // Turn right if the target is to the right
                    else // if(error < 0.0)
                    {
                        //std::cout << "Turning right\n";
                    }
                    cmd.trans_v = 0;
                    cmd.angular_v = turnspeed;

                }
                else
                {
                    std::cout << "Entering DRIVE state.\n";
                    cmd.trans_v = 0;
                    cmd.angular_v = 0;
                    totalError_ = 0;
                    lastError_ = 0;
		            state_ = DRIVE;
                }
            }
            else if(state_ == DRIVE) // Use feedback to drive to the target once approximately pointed in the correct direction
            {
                double speed = kDesiredSpeed;

                float distToGoal = std::sqrt(std::pow(target.x - pose.x, 2.0f) + std::pow(target.y - pose.y, 2.0f));
                
                if (distToGoal < slowDownDistance) {
                    speed = kMinSpeed;
                }

                //go slower if the angle error is greater
		        speed *= std::cos(error);
                //don't go backwards
                speed = std::max(0.0f, float(speed));

                cmd.trans_v = speed;

                //pid control the angular v based on angle error
                cmd.angular_v = error * kPTurnGain;
                //angular velocity must not exceed 1.5 the desired turnspeed
                if (cmd.angular_v >= 0.0) {
                    cmd.angular_v = std::min(std::abs(cmd.angular_v), kTurnMaxSpeed * 1.5f);
                } else {
                    cmd.angular_v = -std::min(std::abs(cmd.angular_v), kTurnMaxSpeed * 1.5f);
                }
		}
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
        }
        
        return cmd;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const bot_msgs::timestamp_t::ConstPtr& timesync){
        timesync_initialized_ = true;
        time_offset = timesync->data->utime-utime_now();
    }
    
    bool sleep = true;
    void handlePath(const bot_msgs::robot_path_t::ConstPtr& path)
    {
        /////// TODO: Implement your handler for new paths here ////////////////////

        // Don't updating the path if the path is being received too quickly
        if(num_points_reached != 3 && !haveReachedTarget()) return;

        num_points_reached = 0;

        targets_ = path->data->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->data->utime << "\n";
    	for(auto pose : targets_){
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}std::cout << "\n";

        assignNextTarget();

        confirm.utime = now();
        confirm.creation_time = path->data->utime;
        confirm.channel = channel;

        //confirm that the path was received
        pubs[MESSAGE_CONFIRMATION_CHANNEL].publish(confirm);
    }
    
    void handleOdometry(const bot_msgs::odometry_t::ConstPtr& odometry)
    {
        /////// TODO: Implement your handler for new odometry data ////////////////////
        
        pose_xyt_t pose;
        pose.utime = odometry->data->utime;
        pose.x = odometry->data->x;
        pose.y = odometry->data->y;
        pose.theta = odometry->data->theta;
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const bot_msgs::pose_xyt_t::ConstPtr& pose)
    {
        /////// TODO: Implement your handler for new pose data ////////////////////    
        computeOdometryOffset(pose->data);
    }
    
private:
    
    enum State
    {
        TURN,
        DRIVE,
    };
    
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;
    
    // Error terms for the current target
    State state_;
    double lastError_;      // for D-term
    double totalError_;     // for I-term

    int64_t time_offset;

    bool timesync_initialized_;

    message_received_t confirm;

    ros::NodeHandle* nodeInstance_;                              // Instance of ros NodeHandle for communication
    std::vector<ros::Subscriber> subs;
    std::map<std::string, ros::Publisher> pubs;     // map of topic names to publishers

    int64_t now(){
	return utime_now()+time_offset;
    }

    bool haveReachedTarget(void)
    {
        const float kPosTolerance = 0.1f;
	    const float kFinalPosTolerance = 0.05f;

        //tolerance for intermediate waypoints can be more lenient
    	float tolerance = (targets_.size() == 1) ? kFinalPosTolerance : kPosTolerance;
        
        // There's no target, so we're there by default.
        if(targets_.empty())
        {
            return true;
        }
        // If there's no odometry, then we're nowhere, so we couldn't be at a target
        if(odomTrace_.empty())
        {
            return false;
        }
        
        pose_xyt_t target = targets_.back();
        pose_xyt_t pose = currentPose();
        
        float xError = std::abs(target.x - pose.x);
        float yError = std::abs(target.y - pose.y);
        
        return (state_ == DRIVE) && (xError < tolerance) && (yError < tolerance);
    }
    
    bool assignNextTarget(void)
    {
        // If there was a target, remove it
        if(!targets_.empty())
        {
            targets_.pop_back();
            num_points_reached++;
            std::cout << "removed target\n";
        }
        
        // Reset all error terms when switching to a new target
        lastError_ = 0.0f;
        totalError_ = 0.0f;
        state_ = TURN;
        
        return !targets_.empty();
    }
    
    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
        
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }
    
    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        
        return pose;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nodeInstance;
    MotionController controller(&nodeInstance);

    subs.push_back(nodeInstance_->subscribe(ODOMETRY_CHANNEL, 1000, MotionController::handleOdometry));
    subs.push_back(nodeInstance_->subscribe(SLAM_POSE_CHANNEL, 1000, MotionController::handlePose));
    subs.push_back(nodeInstance_->subscribe(CONTROLLER_PATH_CHANNEL, 1000, MotionController::handlePath));
    subs.push_back(nodeInstance_->subscribe(MBOT_TIMESYNC_CHANNEL, 1000, MotionController::handleTimesync));

    pubs[ODOMETRY_CHANNEL] = nodeInstance_.advertise<bot_msgs::odometry_t>(ODOMETRY_CHANNEL, 1000);
    pubs[SLAM_POSE_CHANNEL] = nodeInstance_.advertise<bot_msgs::pose_xyt_t>(SLAM_POSE_CHANNEL, 1000);
    pubs[CONTROLLER_PATH_CHANNEL] = nodeInstance_.advertise<bot_msgs::robot_path_t>(CONTROLLER_PATH_CHANNEL, 1000);
    pubs[MBOT_TIMESYNC_CHANNEL] = nodeInstance_.advertise<bot_msgs::timestamp_t>(MBOT_TIMESYNC_CHANNEL, 1000);

    signal(SIGINT, exit);
    
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            mbot_motor_command_t cmd = controller.updateCommand();
            
            pubs[MBOT_MOTOR_COMMAND_CHANNEL].publish(cmd);
    	}
    }
    
    return 0;
}
