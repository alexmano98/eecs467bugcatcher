#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// DONE: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    // iterate through each cell in the obstable map
    float max = sqrt(height_ * height_ + width_ * width_);
    float min_dist, dist;
    for(int oy = 0; oy < height_; ++oy) {
        for(int ox = 0; ox < width_; ++ox) {
            // if itself is an obstable, min dist to obstacle is 0
            if (map(ox, oy) > 0) {
                cells_[cellIndex(ox, oy)] = 0;
            } else {
                // for each cell, iterate through each cell in the occupancy grid map
                min_dist = max;
                for(int gy = 0; gy < height_; ++gy) {
                    for(int gx = 0; gx < width_; ++gx) {
                        if (map(gx, gy) > 0) { // if obstacle found
                            // compute euclidian distance to the cell
                            dist = (gy - oy) * (gy - oy) + (gx - ox) * (gx - ox);
                            if (dist < min_dist) {
                                min_dist = dist;
                            } // TODO: add heuristics to make this more efficient
                        }
                    }
                }
                cells_[cellIndex(ox, oy)] = sqrt(min_dist) * map.metersPerCell();
            }
            // if (oy == 150 && ox == 149) {
            //     std::cout << "Cell odds at (" << oy << ',' << ox << ") is: " << cells_[cellIndex(ox, oy)] << std::endl;
            // }
        }
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
