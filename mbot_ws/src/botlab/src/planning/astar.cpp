#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <algorithm>
#include "float.h"

static const double EPSILON = 1e-6;
static const double ANGLE_EPS = M_PI / 16;

// Struct for a* algorithm
struct a_star_node {
    float g, f;
    int x_parent, y_parent;
    float theta;
};

struct Cell {
    int x, y;
    float f;
    Cell(int x_in, int y_in, float f_in)
        : x(x_in), y(y_in), f(f_in) { }
};

// max comparator for a min heap priority queue
struct CellComparator {
    bool operator () (const Cell & c1, const Cell &c2) const {
        return c1.f > c2.f;
    }
};

// Using the Manhattan distance to approximate the distance from a point to the goal
// Eucleadian distance is a better lower bound than Manhattan distance
float calculate_h(int x, int y, int goal_x, int goal_y){
    return sqrt ( (x - goal_x) * (x - goal_x) + (y - goal_y) * (y-goal_y) );
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// DONE: Implement your A* search here //////////////////////////
    // FOLLOWING THIS ALGORITHM https://www.geeksforgeeks.org/a-search-algorithm/
    // std::cout << "Searching for path\n";
    robot_path_t path;
    // path.utime = start.utime; // TODO: does not seem like time matters
    path.path.push_back(start);    
    path.path_length = path.path.size();

    // convert poses to occupancy grid coordinates
    Point<int> start_pose_map = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Point<int> goal_pose_map = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    int start_x = start_pose_map.x;
    int start_y = start_pose_map.y;
    int goal_x = goal_pose_map.x;
    int goal_y = goal_pose_map.y;

    // std::cout << "Start " << start_x << ' ' << start_y << "\n";
    // std::cout << "Goal " << goal_x << ' ' << goal_y << "\n";

    // std::cout << "checking goal\n";
    // If the goal is unreachable or goal is the same as start, then a path with just the initial pose is returned
    if (!distances.isCellInGrid(start_x, start_y)) {
        // std::cout << "Start " << start_x << ' ' << start_y << " not in obstacle grid\n";
        return path;
    }
    if (!distances.isCellInGrid(goal_x, goal_y)) {
        // std::cout << "Goal " << goal_x << ' ' << goal_y << " not in obstacle grid\n";
        return path;
    }
    if (start_x == goal_x && start_y == goal_y) {
        // std::cout << "start and goal poses are the same \n";
        return path;
    }
    // std::cout << "distance to obstable from start: " << distances(start_x, start_y) << std::endl;
    // std::cout << "distance to obstable from goal: " << distances(goal_x, goal_y) << std::endl;
    if (distances(start_x, start_y) < params.minDistanceToObstacle || // determine whether cell is occupied
    distances(goal_x, goal_y) < params.minDistanceToObstacle) { // either the start or goal is blocked
        // std::cout << "goal unreachablet\n";
        return path;
    }
    // std::cout << "Valid start and goal\n";

    // Vectors for the open and closed structures for the algorithm
    std::priority_queue<Cell, std::vector<Cell>, CellComparator> open_list;
    // closed list should be a 2D vector of bools
    std::vector<std::vector<bool>> closed(distances.heightInCells(), std::vector<bool>(distances.widthInCells(), false));

    // Stores pairs of poses and current distance from the start
    a_star_node node;
    node.f = FLT_MAX;
    std::vector<std::vector<a_star_node>> cell_details(distances.heightInCells(), std::vector<a_star_node>(distances.widthInCells(), node));

    // Initialising the parameters of the starting node 
    cell_details[start_y][start_x].f = 0.0; 
    cell_details[start_y][start_x].g = 0.0; 
    cell_details[start_y][start_x].y_parent = start_y; 
    cell_details[start_y][start_x].x_parent = start_x;
    // cell_details[start_y][start_x].dist = 0; 
    
    // Add start node to open
    open_list.push(Cell(start_x, start_y, 0));

    bool found_path = false;

    // Use min heap priority queue to rank from smallest f; While open is not empty, keep going
    while (!open_list.empty()) {
        // Pop off the node and erase from open
        Cell c = open_list.top();
        open_list.pop();
        int x = c.x, y = c.y;
        double new_dist;
        closed[y][x] = true; // Add this vertex to the closed list

        // DONE: Add the 8 surrounding points to open as long as their value in 'distances' > 0
        // If the value in 'distances' is less than minDistanceToObstacle, then there is an obstacle
        //----------- 1st Successor (South) ------------ 
        if (distances.isCellInGrid(x, y - 1)) {
            // If the destination cell is the same as the current successor 
            if ((y - 1) == goal_y && x == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y - 1][x].y_parent = y; 
                cell_details[y - 1][x].x_parent = x; 
                cell_details[y - 1][x].theta = - M_PI / 2; // tODO: theta points to the next pose in the sequence
                found_path = true; // The destination cell is found
                break;
            }
            new_dist = distances(x, y - 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y - 1][x]) { 
                    float g = cell_details[y][x].g + calculate_h(x, y - 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x, y - 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y-1][x].f == FLT_MAX || cell_details[y - 1][x].f > f) { 
                        open_list.push(Cell(x, y - 1, f));
                        // Update the details of this cell 
                        cell_details[y - 1][x].f = f; 
                        cell_details[y - 1][x].g = g; 
                        cell_details[y - 1][x].y_parent = y; 
                        cell_details[y - 1][x].x_parent = x; 
                        cell_details[y - 1][x].theta = - M_PI / 2;
                    } 
                } 
            }
        }

        //----------- 2nd Successor (North) ------------ 
        if (distances.isCellInGrid(x, y + 1)) {
            // If the destination cell is the same as the current successor 
            if ((y + 1) == goal_y && x == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y + 1][x].y_parent = y; 
                cell_details[y + 1][x].x_parent = x; 
                cell_details[y + 1][x + 1].theta = M_PI / 2; // TODO: theta points to the next pose in the sequence
                found_path = true; // The destination cell is found
                break;
            }
            new_dist = distances(x, y + 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y + 1][x]) { 
                    float g = cell_details[y][x].g + calculate_h(x, y + 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x, y + 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        // std::cout << "adding a cost of " << pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent) << std::endl;
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
                    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y + 1][x].f == FLT_MAX || cell_details[y + 1][x].f > f) { 
                        open_list.push(Cell(x, y + 1, f));
                        // Update the details of this cell 
                        cell_details[y + 1][x].f = f; 
                        cell_details[y + 1][x].g = g; 
                        cell_details[y + 1][x].y_parent = y; 
                        cell_details[y + 1][x].x_parent = x; 
                        cell_details[y + 1][x + 1].theta = M_PI / 2;
                    } 
                } 
            }  
        }

        //----------- 3rd Successor (East) ------------ 
        if (distances.isCellInGrid(x + 1, y)) {
            // If the destination cell is the same as the current successor 
            if (y == goal_y && (x + 1) == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y][x + 1].y_parent = y; 
                cell_details[y][x + 1].x_parent = x; 
                cell_details[y][x + 1].theta = 0; // TODO: theta points to the next pose in the sequence
                found_path = true;                // The destination cell is found
                break; 
            } 
            new_dist = distances(x + 1, y);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y][x + 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x + 1, y, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x + 1, y, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y][x + 1].f == FLT_MAX || cell_details[y][x + 1].f > f) { 
                        // std::cout << " g : " << g << " h: " << h << " f: " << f;
                        // std::cout << " current pos " << x + 1 << ' ' << y << std::endl;
                        open_list.push(Cell(x + 1, y, f));
                        // Update the details of this cell 
                        cell_details[y][x + 1].f = f; 
                        cell_details[y][x + 1].g = g; 
                        cell_details[y][x + 1].y_parent = y; 
                        cell_details[y][x + 1].x_parent = x; 
                        cell_details[y][x + 1].theta = 0;
                    } 
                } 
            }  
        }

        //----------- 4th Successor (West) ------------ 
        if (distances.isCellInGrid(x - 1, y)) {
            // If the destination cell is the same as the current successor 
            if (y == goal_y && (x - 1) == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y][x - 1].y_parent = y; 
                cell_details[y][x - 1].x_parent = x; 
                cell_details[y][x - 1].theta = M_PI; // TODO: theta points to the next pose in the sequence
                found_path = true;                    // The destination cell is found
                break;
            } 
            new_dist = distances(x - 1, y);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y][x - 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x - 1, y, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x - 1, y, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y][x - 1].f == FLT_MAX || cell_details[y][x - 1].f > f) { 
                        open_list.push(Cell(x - 1, y, f));
                        // Update the details of this cell 
                        cell_details[y][x - 1].f = f; 
                        cell_details[y][x - 1].g = g; 
                        cell_details[y][x - 1].y_parent = y; 
                        cell_details[y][x - 1].x_parent = x; 
                        cell_details[y][x - 1].theta = M_PI;
                    } 
                } 
            }  
        }

        //----------- 5th Successor (North-West) ------------ 
        if (distances.isCellInGrid(x - 1, y + 1)) {
            // If the destination cell is the same as the current successor 
            if ((y + 1) == goal_y && (x - 1) == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y + 1][x - 1].y_parent = y; 
                cell_details[y + 1][x - 1].x_parent = x; 
                cell_details[y + 1][x - 1].theta = 3 * M_PI / 4; // TODO: theta points to the next pose in the sequence
                found_path = true;                // The destination cell is found
                break;
            } 
            new_dist = distances(x - 1, y + 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y + 1][x - 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x - 1, y + 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x - 1, y + 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y + 1][x - 1].f == FLT_MAX || cell_details[y + 1][x - 1].f > f) { 
                        open_list.push(Cell(x - 1, y + 1, f));
                        // Update the details of this cell 
                        cell_details[y + 1][x - 1].f = f; 
                        cell_details[y + 1][x - 1].g = g; 
                        cell_details[y + 1][x - 1].y_parent = y; 
                        cell_details[y + 1][x - 1].x_parent = x; 
                        cell_details[y + 1][x - 1].theta = 3 * M_PI / 4;
                    } 
                } 
            }  
        }

        //----------- 6th Successor (South-East) ------------ 
        if (distances.isCellInGrid(x - 1, y - 1)) {
            // If the destination cell is the same as the current successor 
                if ((y - 1) == goal_y && (x - 1) == goal_x) { 
                    // Set the Parent of the destination cell 
                    cell_details[y - 1][x - 1].y_parent = y; 
                    cell_details[y - 1][x - 1].x_parent = x; 
                    cell_details[y - 1][x - 1].theta = - 3 * M_PI / 4; // TODO: theta points to the next pose in the sequence
                    found_path = true;                    // The destination cell is found
                    break; 
                } 
            new_dist = distances(x - 1, y - 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y - 1][x - 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x - 1, y - 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x - 1, y - 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y - 1][x - 1].f == FLT_MAX || cell_details[y - 1][x - 1].f > f) { 
                        open_list.push(Cell(x - 1, y - 1, f));
                        // Update the details of this cell 
                        cell_details[y - 1][x - 1].f = f; 
                        cell_details[y - 1][x - 1].g = g; 
                        cell_details[y - 1][x - 1].y_parent = y; 
                        cell_details[y - 1][x - 1].x_parent = x; 
                        cell_details[y - 1][x - 1].theta = - 3 * M_PI / 4;
                    } 
                } 
            }  
        }

        //----------- 7th Successor (North-East) ------------ 
        if (distances.isCellInGrid(x + 1, y + 1)) {
            // If the destination cell is the same as the current successor 
            if ((y + 1) == goal_y && (x + 1) == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y + 1][x + 1].y_parent = y; 
                cell_details[y + 1][x + 1].x_parent = x; 
                cell_details[y + 1][x + 1].theta = M_PI / 4; // TODO: theta points to the next pose in the sequence
                found_path = true;                // The destination cell is found
                break; 
            } 
            new_dist = distances(x + 1, y + 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y + 1][x + 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x + 1, y + 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x + 1, y + 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y + 1][x + 1].f == FLT_MAX || cell_details[y + 1][x + 1].f > f) { 
                        open_list.push(Cell(x + 1, y + 1, f));
                        // Update the details of this cell 
                        cell_details[y + 1][x + 1].f = f; 
                        cell_details[y + 1][x + 1].g = g; 
                        cell_details[y + 1][x + 1].y_parent = y; 
                        cell_details[y + 1][x + 1].x_parent = x; 
                        cell_details[y + 1][x + 1].theta = M_PI / 4;
                    } 
                } 
            }
        }

        //----------- 8th Successor (North-West) ------------ 
        if (distances.isCellInGrid(x - 1, y + 1)) {
           
            // If the destination cell is the same as the current successor 
            if ((y + 1) == goal_y && (x - 1) == goal_x) { 
                // Set the Parent of the destination cell 
                cell_details[y + 1][x - 1].y_parent = y; 
                cell_details[y + 1][x - 1].x_parent = x; 
                // cell_details[y + 1][x - 1].dist = parent_dist;
                cell_details[y + 1][x - 1].theta = 3 * M_PI / 4; // TODO: theta points to the next pose in the sequence
                found_path = true;                // The destination cell is found
                break; 
            } 
            new_dist = distances(x - 1, y + 1);
            // Only process this cell if this is a valid one 
            if (new_dist - params.minDistanceToObstacle > EPSILON) { 
                // If the successor is already on the closed list or if it is blocked, then ignore it. 
                if (!closed[y + 1][x - 1]) { 
                    float g = cell_details[y][x].g + calculate_h(x - 1, y + 1, x, y) * distances.metersPerCell(); 
                    float h = calculate_h(x - 1, y + 1, goal_x, goal_y) * distances.metersPerCell(); 
                    float f = g + h; 
                    
                    // add cost to distance too close to obstable?
                    if (new_dist < params.maxDistanceWithCost) {
                        f += pow(params.maxDistanceWithCost - new_dist, params.distanceCostExponent);
                    }
    
                    // If it isn’t on the open list, add it to the open list. Make the current square 
                    // the parent of this square. Record the f, g, and h costs of the square cell 
                    // If it is on the open list already, check to see if this path to that square is better, 
                    // using 'f' cost as the measure. 
                    if (cell_details[y + 1][x - 1].f == FLT_MAX || cell_details[y + 1][x - 1].f > f) { 
                        open_list.push(Cell(x - 1, y + 1, f));
                        // Update the details of this cell 
                        cell_details[y + 1][x - 1].f = f; 
                        cell_details[y + 1][x - 1].g = g; 
                        cell_details[y + 1][x - 1].y_parent = y; 
                        cell_details[y + 1][x - 1].x_parent = x; 
                        cell_details[y + 1][x - 1].theta = 3 * M_PI / 4;
                    } 
                } 
            }
        }
            
    }

    if (found_path) {
        int parent_x = cell_details[goal_y][goal_x].x_parent;
        int parent_y = cell_details[goal_y][goal_x].y_parent;
        float parent_theta = cell_details[goal_y][goal_x].theta;

        std::vector<pose_xyt_t> temp_path;
        temp_path.push_back(goal);

        // std::cout << "parent cells are " << parent_x << ' ' << parent_y << std::endl;
        path.path.insert(path.path.begin() + 1, goal); // insert goal behind start
        // std::cout << "inserted " << goal_x << ' ' << goal_y << " to path\n";

        while (parent_x != start_x || parent_y != start_y) {
            pose_xyt_t dest;
            // Converts grid waypoint to pose waypoint
            Point<double> pose = grid_position_to_global_position(Point<double>(parent_x, parent_y), distances);
            dest.x = pose.x;
            dest.y = pose.y;
            dest.theta = parent_theta;
            temp_path.push_back(dest);

            parent_theta = cell_details[parent_y][parent_x].theta;
            int prev_x = parent_x;
            // retrieve next waypoint
            parent_x = cell_details[parent_y][parent_x].x_parent;
            parent_y = cell_details[parent_y][prev_x].y_parent;
        }
        // reverse the path
        std::reverse(temp_path.begin(),temp_path.end());

        // remove uneccessary waypoints: Uneccessary waypoints are any points that don't change heading
        // initialize the path to start at start point with heading towards next waypoint
        path.path.clear();
        path.path.reserve(temp_path.size() + 1);
        pose_xyt_t current = start;
        current.theta = parent_theta;
        path.path.push_back(start);
        pose_xyt_t prev_pose = current;
        float prev_theta = current.theta; // what should the start theta be?
        for(size_t i = 0; i < temp_path.size(); i++)
        {
            current = temp_path[i];
            float dist = sqrt((current.x - prev_pose.x) * (current.x - prev_pose.x) + 
                         (current.y - prev_pose.y) * (current.y - prev_pose.y));
            if(fabs(current.theta - prev_theta) > ANGLE_EPS || dist > 0.1 || i == temp_path.size() - 1)
            {
                // std::cout << "thetas: " << current.theta << ", " << prev_theta << std::endl;
                // std::cout << "x,y: " << current.x << ", " << current.y << std::endl;
                path.path.push_back(current);
                prev_theta = current.theta;
                prev_pose = current;
            }
        }

        path.path_length = path.path.size();
    }

    // TODO REMOVE THIS LATER MAYBE
    // path.path.erase(path.path.begin());
    // path.path_length--;

    return path;
}