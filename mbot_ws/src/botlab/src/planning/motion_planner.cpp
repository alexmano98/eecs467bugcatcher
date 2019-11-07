#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>
#include <iostream>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
    prev_goal.x = 0;
    prev_goal.y = 0;
    prev_goal.theta = 0;
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
    prev_goal.x = 0;
    prev_goal.y = 0;
    prev_goal.theta = 0;
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    // if(!isValidGoal(goal))
    // {
    //     robot_path_t failedPath;
    //     failedPath.utime = utime_now();
    //     failedPath.path_length = 1;
    //     failedPath.path.push_back(start);
        
    //     std::cout << "Motion planner failed to find path from :" << start.x << ' ' << start.y << " to " <<
    //     goal.x << ' ' << goal.y << std::endl << " (invalid goal) \n";
    //     std::cout << "INFO: path rejected due to invalid goal\n";        

    //     return failedPath;
    // }
    
    std::cout << "searching for path with A*\n";
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    // std::cout << "distance from previous goal: " << distanceFromPrev << std::endl;

    // if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    if(num_frontiers != 1 && distanceFromPrev < searchParams_.minDistanceToObstacle) {
        // std::cout << "more than 1 frontier, don't go to the one closest" << "\n";
        std::cout << "dist from " << prev_goal.x << ' ' << prev_goal.y << " to " << goal.x << ' '
        << goal.y << " is " << distanceFromPrev << std::endl;
        return false;
    }

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        // std::cout << "goal dist to obstacle: " << distances_(goalCell.x, goalCell.y) << "\n";
        // std::cout << "cell of goalcell: " << goalCell.x << " " << goalCell.y << std::endl;
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    
    std::cout << goalCell.x << ' ' << goalCell.y << " Not in grid?\n";
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// DONE: Implement your test for a safe path here //////////////////
    std::cout << "DEBUG motion planner is path safe\n";
    for (auto pose : path.path) {
        // check if each pose is at or too close to an obstable
        // get x, y coordinates in map coord of the pose
        Point<int> cell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), distances_);
        if (distances_(cell.x, cell.y) <  searchParams_.minDistanceToObstacle) {
            std::cout << "Pose at " << pose.x << ' ' << pose.y << " is unsafe\n";
            return false;
        }
    }
    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle =  params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 5;
}
