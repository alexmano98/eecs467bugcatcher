#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>

// TODO: Tune this value
static const int8_t DELTA_LOGODDS = 8; // lecture 6 slide 10 used difference of 4


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if(!initialized_)
    {
        previousPose_ = pose;
    }
    
    MovingLaserScan movingScan(scan, previousPose_, pose);
    
    //////////////// DONE: Implement your occupancy grid algorithm here ///////////////////////
    for (size_t i = 0; i < movingScan.size(); ++i) {
        updateEndpoint(movingScan[i], map);
        updateRay(movingScan[i], map);
        // std::cout << "updated moving scan number: " << i << std::endl;
    }
}


void Mapping::updateEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// DONE: Implement this function //////////////////
    Point<double> global_pose = Point<double>((ray.origin.x + ray.range * cos(ray.theta)), (ray.origin.y + ray.range * sin(ray.theta)));
    Point<int> cell_coord = global_position_to_grid_cell(global_pose, map);
    // std::cout << "updating endpoint (x, y): " << cell_coord.x << " " << cell_coord.y << std::endl;
    increaseCellOdds(cell_coord.x, cell_coord.y, map);
}


void Mapping::updateRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// DONE: Implement this function //////////////////
    float deltax = ray.range * cos(ray.theta);
    float deltay = ray.range * sin(ray.theta);
    float deltaerr = fabs(deltay / deltax);
    float error = 0.0;
    // float y = ray.origin.y;
    // std::cout << "laser range: " << ray.range << std::endl;
    // std::cout << "ray theta: " << ray.theta << std::endl;
    for (float r = 0; r < ray.range - map.metersPerCell(); r += map.metersPerCell()) { // TODO: Tune this - 0.025
        float x = ray.origin.x + cos(ray.theta) * r;
        float y = ray.origin.y + sin(ray.theta) * r;
        // decreaseCellOdds(static_cast<int>(round((x + map.widthInMeters() / 2) * map.cellsPerMeter())),
        //  static_cast<int>(round((y + map.heightInMeters() / 2) * map.cellsPerMeter())), map);
        Point<int> cell_coord = global_position_to_grid_cell(Point<double>(x, y), map);
        // decreaseCellOdds(static_cast<int>(round((x - map.originInGlobalFrame().x) * map.cellsPerMeter())),
        //  static_cast<int>(round((y - map.originInGlobalFrame().y) * map.cellsPerMeter())), map);
        // std::cout << "decreasing cell odds: " << cell_coord.x << " " << cell_coord.y << std::endl;
        decreaseCellOdds(cell_coord.x, cell_coord.y, map);
        error += deltaerr;
        
        // if (error >= map.metersPerCell() / 2) {
        //     y += (map.metersPerCell() * abs(sin(ray.theta)) / sin(ray.theta));
        //     error -= map.metersPerCell();               
        // }
    }
}


void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// DONE: Implement this function //////////////////
    int odds = map.logOdds(x, y);
    if (odds - DELTA_LOGODDS < -128) {
        map.setLogOdds(x, y, -128);
    } else {
        map.setLogOdds(x, y, static_cast<int8_t>(odds - DELTA_LOGODDS));
    }
}


void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// DONE: Implement this function //////////////////
    int odds = map.logOdds(x, y);
    // std::cout << "Log oddds: " << odds << std::endl;
    if (odds + DELTA_LOGODDS > 127) {
        map.setLogOdds(x, y, 127);
        // std::cout << "x, y: " << x << " " << y << std::endl;
    } else {
        map.setLogOdds(x, y, static_cast<int8_t>(odds + DELTA_LOGODDS));
    }
}
