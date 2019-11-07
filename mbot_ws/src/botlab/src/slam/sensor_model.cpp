#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <common/grid_utils.hpp>
#include <common/angle_functions.hpp>

// TODO: Tune these values
// Ray terminating before know obstacle is less terrible than terminating after
// increasing slopes == smaller standard deviation
// small std dev --> closer and more accurate model
// However, we need a bigger std dev when it deviates from course
static const int PEAK_DISTRIBUTION = 4;    // 2
static const int SLOPE_BEFORE = 16;         // 28
static const int SLOPE_AFTER = 16;          // 60

#define OBSTACLE_THRESHOLD 0
// anything more than 0 means that there is more likely to be an obstacle


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// DONE: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    double scanLikelihood = 0.0;
    // Iterate throught the laser scan and determine if the scan has stopped before, on, or after the first obstacle in its path
    for(int32_t i = 0; i < scan.num_ranges; i++){
        float theta = sample.pose.theta - scan.thetas[i];
        Point<int> cell_coord = global_position_to_grid_cell(Point<double>((sample.pose.x + scan.ranges[i] * cos(theta)), (sample.pose.y + scan.ranges[i] * sin(theta))), map);
        int x_scan = cell_coord.x;
        int y_scan = cell_coord.y;   
        
        // Perform Breshenham's line alg (walk along ray)
        float deltax = scan.ranges[i] * cos(theta);
        float deltay = scan.ranges[i] * sin(theta);
        float deltaerr = fabs(deltay / deltax);
        float error = 0.0;
        // float y = sample.pose.y;

        bool reached_obstacle = false;

        // walk along ray to check if it intersects obstacle on grid before the endpoint
        for (float r = 0; r < scan.ranges[i] - map.metersPerCell(); r += map.metersPerCell()) {
            float x = sample.pose.x + cos(theta) * r;
            float y = sample.pose.y + sin(theta) * r;
            Point<int> cell = global_position_to_grid_cell(Point<double>(x, y), map);
            // Check if the laser has reached an obstacle before end of scan
            if(map.logOdds(cell.x, cell.y) > OBSTACLE_THRESHOLD){
                float dist_to_obstacle = scan.ranges[i] - r;
                scanLikelihood += (PEAK_DISTRIBUTION - SLOPE_BEFORE * dist_to_obstacle);
                // scanLikelihood -= 8;       // terminates before a knwon obstacle 
                reached_obstacle = true;
                break;
            }
            error += deltaerr;
            // if (error >= map.metersPerCell() / 2) {
            //     y += (map.metersPerCell() * abs(sin(theta)) / sin(theta));
            //     error -= map.metersPerCell();
            // }
        }
        if(reached_obstacle) continue;

        // Check if the scan terminates at an obstacle
        if(map.logOdds(x_scan, y_scan) > OBSTACLE_THRESHOLD){
            scanLikelihood += PEAK_DISTRIBUTION;
            // scanLikelihood -= 4;

        // Otherwise, scan terminates after obstable has been reached
        } else {
            // y = sample.pose.y + scan.ranges[i] * sin(theta);
            error = 0.0;
            // keep going along the ray and search for obstacle
            for (float r = scan.ranges[i] + map.metersPerCell(); r < 2 * scan.ranges[i]; r += map.metersPerCell()) {
                float x = sample.pose.x + cos(theta) * r;
                float y = sample.pose.y + cos(theta) * r;
                Point<int> cell = global_position_to_grid_cell(Point<double>(x, y), map);
                // Check if the laser has reached an obstacle after end of scan
                if(map.logOdds(cell.x, cell.y) > OBSTACLE_THRESHOLD){
                    float dist_to_obstacle = r - scan.ranges[i];
                    scanLikelihood += (PEAK_DISTRIBUTION - SLOPE_AFTER * dist_to_obstacle);
                    // scanLikelihood -= 12;       // terminates after an obstacle 
                    break;
                }
                error += deltaerr;
                // if (error >= map.metersPerCell() / 2) {
                //     y += (map.metersPerCell() * abs(sin(theta)) / sin(theta));
                //     error -= map.metersPerCell();
                // }
            }
        }
        
    }

    return scanLikelihood;
}


    // Perform Breshenham's line alg (walk along ray)
    // float deltax = scan.ranges[i] * cos(theta);
    //     float deltay = scan.ranges[i] * sin(theta);
    //     float deltaerr = fabs(deltay / deltax);
    //     float error = 0.0;
    //     float y = sample.pose.y;

    //     bool reached_obstacle = false;

    //     // walk along ray, if it intersects obstacle on grid
    //     for (float r = 0; r < scan.ranges[i] - map.metersPerCell() / 2; r += map.metersPerCell()) {
    //         float x = sample.pose.x + cos(theta) * r;
    //         Point<int> cell = global_position_to_grid_cell(Point<double>(x, y), map);
    //         // Check if the laser has reached an obstacle before end of scan
    //         if(map.logOdds(cell.x, cell.y) > OBSTACLE_THRESHOLD){
    //             scanLikelihood -= 12;    // ray terminates before knwon obstacle 
    //             reached_obstacle = true;
    //             break;
    //         }
    //         error += deltaerr;
    //         if (error >= map.metersPerCell() / 2) {
    //             y += (map.metersPerCell() * abs(sin(theta)) / sin(theta));
    //             error -= map.metersPerCell();
    //         }
    //     }
    //     if(reached_obstacle) continue;
        
    //     // Check if the scan terminates at an obstacle
    //     if(map.logOdds(x_scan, y_scan) > OBSTACLE_THRESHOLD){
    //         scanLikelihood -= 4;

    //     // Otherwise, scan terminates after obstable has been reached
    //     } else {
    //         scanLikelihood -= 8; // 12
    //     }