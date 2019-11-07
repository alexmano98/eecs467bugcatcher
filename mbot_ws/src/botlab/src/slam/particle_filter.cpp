#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <cstdlib>
#include <numeric>
#include "limits.h"
#include "float.h"
#include <algorithm>
#include <iostream>

// how many particles to average based on weight
static const size_t SAMPLE_SIZE = 8;

// for sorting (max to min)
bool compareParticles (const particle_t &p1, const particle_t &p2) {
    return p1.weight > p2.weight;
}

// for finding the max
// bool compareParticles (const particle_t &p1, const particle_t &p2) {
//     return p1.weight < p2.weight;
// }

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.reserve(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// DONE: Implement your method for initializing the particles in the particle filter /////////////////
    actionModel_ = ActionModel();
    sensorModel_ = SensorModel();
    // initialize the particles
    for (int i = 0; i < kNumParticles_; i++) {
        particle_t init;
        init.pose = pose;
        init.parent_pose = pose;
        init.weight = (float) 1 / (float)kNumParticles_;
        posterior_.push_back(init);
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If robot didn't move do nothing.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    // std::cout << "Robot at " << odometry.x << ' ' << odometry.y << " moved: " << hasRobotMoved << std::endl;
    
    if(hasRobotMoved)
    {
        std::vector<particle_t> prior = resamplePosteriorDistribution();
        std::vector<particle_t> proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// DONE: Implement your algorithm for resampling from the posterior distribution ///////////////////
    // Resample the points
    std::vector<particle_t> prior;
    prior.reserve(posterior_.size());

    // For each point in posterior_, resample based on weights and place into prior
    for(size_t i = 0; i < posterior_.size(); i++){
        // Sample from posterior
        float alpha = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
        // std::cout << "random sample alph = " << alpha << ' ';
        float weight = 0;
        size_t indx = 0;
        while(weight < alpha && indx < posterior_.size()){
            weight += posterior_[indx].weight;
            indx++;
        }
        indx--; // off by one error
        // assert(weight >= alpha);
        // std::cout << "sampling from index " << indx << std::endl;
        // std::cout << "prior contains: " << posterior_[indx].pose.x << ' ' << posterior_[indx].pose.y << ' ' << posterior_[indx].pose.theta << '\n';
        prior.push_back(posterior_[indx]);
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// DONE: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    // Apply action model
    std::vector<particle_t> proposal;
    proposal.resize(prior.size());

    // std::cout << "prior size: " << prior.size() << std::endl;
    for(size_t i = 0; i < prior.size(); i++){
        particle_t sample = actionModel_.applyAction(prior[i]);
        proposal[i] = sample;
        // std::cout << "proposal pose: " << sample.pose.x << ' ' << sample.pose.y << std::endl;
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// DONE: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    // Apply the sensor model
    std::vector<particle_t> posterior;
    posterior.reserve(proposal.size());

    std::vector<double> weights;
    weights.reserve(proposal.size());
    for(size_t i = 0; i < proposal.size(); i++){
        double likelihood = sensorModel_.likelihood(proposal[i], laser, map);
        // std::cout << "Likelihood " << likelihood << std::endl;
        weights.push_back(likelihood);
    }
    // std::cout << std::endl;

    // Normalize weights 
    double max = - FLT_MAX;
    for (double w : weights) {
        // std::cout << "weights " << w << ' ';
        if (w > max) {
            max = w;
        }
    }
    // std::cout << std::endl; 
    // Shift all of weights up by max to avoid underflowing
    for(size_t i = 0; i < weights.size(); i++){
        // std::cout << "i: " << i << " max: " << max << std::endl;
        // std::cout << "before: " << weights[i] << std::endl;
        weights[i] -= max;
        // std::cout << "after: " << weights[i] << "\n\n";
        weights[i] = exp(weights[i]);
    }

    double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    // std::cout << "Sum: " << sum << std::endl << std::endl;

    for(size_t i = 0; i < weights.size(); i++){
        double new_weight = weights[i] / sum;
        // std::cout << "before: " << weights[i] <<"\nafter: " << new_weight << "\n\n";

        // Add updated particle to posterior
        particle_t particle = proposal[i];
        particle.weight = new_weight;
        posterior.push_back(particle);
        // std::cout << "updated posterior: " << particle.pose.x << ' ' << particle.pose.y << ' ' << particle.pose.theta << '\n';
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// DONE: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose /* = posterior[0].pose */;
    // Return the maximum likelihood pose estimate
    std::vector<particle_t> sort_vector(posterior.begin(), posterior.end());
    std::sort(sort_vector.begin(), sort_vector.end(), compareParticles);
    // double max_weight = sort_vector[0].weight;
    // double max_weight = std::max_element(posterior.begin(), posterior.end(), compareParticles)->weight;
    size_t count = 0;
    float sum_x = 0.0, sum_y = 0.0;   
    float sum_cos = 0.0, sum_sin = 0.0; 
    for (const particle_t &p : sort_vector) {   
    // for (const particle_t &p : posterior) {    
        // if (max_weight == p.weight) {
            // std::cout << "Weight " << p.weight << " MAX Weight: " << max_weight << std::endl;
            // compute average x, y, theta
            // std::cout << "evaluating pose at " << p.pose.x << ' ' << p.pose.y << ' ' << p.pose.theta << std::endl;
            sum_x += p.pose.x;
            sum_y += p.pose.y;
            // sum sin and cos for theta
            sum_cos += cos(p.pose.theta);
            sum_sin += sin(p.pose.theta);
            ++count;
            
            if(count >= SAMPLE_SIZE) break;
        // }
    }
    // std::cout << "count: " << count << " num_particles: " << posterior.size() << std::endl;
    pose.x = sum_x / count;
    pose.y = sum_y / count;
    pose.theta = atan2(sum_sin / count, sum_cos / count);
    // std::cout << "updated pose by MCL: " << pose.x << ' ' << pose.y << ' ' << pose.theta << "\n\n";
    return pose;
}
