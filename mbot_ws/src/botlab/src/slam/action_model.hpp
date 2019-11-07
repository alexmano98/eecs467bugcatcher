#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>
#include <vector>
#include <algorithm>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);
    
    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   Whether the action is updated (whether the robot moved).
    */
    bool updateAction(const pose_xyt_t& odometry);
    
    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);
    
private:
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    // Alex's notes
    /*
    Need to add a variable for the previous odometry
    Need to store the action model so that it can be used by applyAction and updated with updateAction
        vars to store for the action model:
            errors: e1, e2, e3
            constants: k1, k2
    */
   bool initialized;
   pose_xyt_t prev_odometry; // odometry from last update
   float delta_s, alpha, delta_phi; // values used by the action model
   // store the previous values to compute the standard deviation
//    std::vector<float> past_alphas; // e1 ~ N(0, K1 alpha)
//    std::vector<float> past_delta_ss; // e2 ~ N(0, K2 Delta s)
//    std::vector<float> past_delta_phi_alpha_diff; // e3 ~ N(0, K1(Delta Phi - Alpha))
//    std::vector<float> error1, error2, error3; // standard deviation of the action model
   float e1, e2, e3;
//    size_t count;
   int64_t time;
};

#endif // SLAM_ACTION_MODEL_HPP
