#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <random>
#include "math.h"
#include <iostream>

// TODO: tune these standard deviation constants
// K1 decides turning error, 1-2 range works ok for driving square
// K2 decides displacement uncertainty, since odometry is theoretically accurate, it needs to be small (0.1)
// However, when the SLAM_POSE deviates from true pose due to turning errors, 
// the K1 and K2 terms are too smal to correct for them
static const float K1 = M_PI / 4; // M_PI / 4
static const float K2 = 0.1; // 0.5
static const float EPSILON = 1e-9;


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    initialized = false;
}

// samples a normal distribution and returns an error
std::default_random_engine generator;
float gaussianError(const float stdv)
{
    std::normal_distribution<float> distribution(0.0, fabs(stdv));
    return distribution(generator);
}
    
bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// DONE: Implement code here to compute a new distribution of the motion of the robot ////////////////

    // If not initialized, set prev_odometry and return
    if(!initialized){
        initialized = true;
        prev_odometry = odometry;
        return false;
    }

    time = odometry.utime;
    if (fabs(prev_odometry.x - odometry.x) < EPSILON && fabs(prev_odometry.y - odometry.y) < EPSILON 
    && fabs(prev_odometry.theta - odometry.theta) < EPSILON) {
        // std::cout << "has not moved from " << prev_odometry.x << ' ' << prev_odometry.y << std::endl;
        return false; // odometry has not changed since the previous reading
    }
    
    // Calculate delta_x, delta_y, delta_s, delta_phi, alpha
    float delta_x = odometry.x - prev_odometry.x;
    float delta_y = odometry.y - prev_odometry.y;
    delta_s = sqrt(delta_x * delta_x + delta_y * delta_y);
    delta_phi = wrap_to_pi(odometry.theta - prev_odometry.theta);
    alpha = wrap_to_pi(atan2(delta_y, delta_x) - prev_odometry.theta);

    prev_odometry = odometry; // boiz we need to update odometry lol...

    // std::cout << "delta s " << delta_s << ", delta phi " << delta_phi << ", alph " << alpha << std::endl;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// DONE: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////

    // Resample from the distributions
    e1 = gaussianError(K1 * alpha);
    e2 = gaussianError(K2 * delta_s);
    e3 = gaussianError(K1 * wrap_to_pi(delta_phi - alpha));
    
    // std::cout << "e1 " << e1 << " e2 " << e2 << " e3 " << e3 << std::endl;
    // e1 = 0.0; //gaussianError(K1 * alpha);
    // e2 = 0.0; //gaussianError(K2 * delta_s);
    // e3 = 0.0; //gaussianError(K1 * (delta_phi - alpha));
    // std::cout << "delta s " << delta_s << ", delta phi " << delta_phi << ", alph " << alpha << std::endl;

    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.pose.utime = time; // set the new time to the time of latest odometry reading
    new_sample.parent_pose = sample.pose; // parent pose is the previous pose
    new_sample.pose.x = sample.pose.x + (delta_s + e2) * cos(sample.pose.theta + alpha + e1);
    new_sample.pose.y = sample.pose.y + (delta_s + e2) * sin(sample.pose.theta + alpha + e1);
    new_sample.pose.theta = wrap_to_pi(sample.pose.theta + delta_phi + e1 + e3); // Lecture 6 slide 6
    // std::cout << "returning new action at " << new_sample.pose.x << ' ' << new_sample.pose.y << ' ' << new_sample.pose.theta << '\n';

    return new_sample;
}
