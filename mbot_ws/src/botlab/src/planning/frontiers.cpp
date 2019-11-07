#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>


static const int ROBOTRADIUS = 0.20;

struct ComparePoint {
     bool operator() (const Point<int> &p1, const Point<int> &p2) const {
         if (p1.x < p2.x) {
             return true;
         } else if (p1.x > p2.x) {
             return false;
         } else {
             return p1.y < p2.y;
         }
     }
 };


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier, 
                              const pose_xyt_t& pose, 
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, 
                                  Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);


// TONY: given the current cell, returns the nearest cell that is free
Point<int> search_to_nearest_free_space(Point<int> position, const OccupancyGrid& map, const MotionPlanner& planner) {

    int limit = static_cast<int> (round(ROBOTRADIUS * 6 * map.cellsPerMeter()));
    Point<int> original_position = position;
    std::queue<Point<int>> neighbors;
    neighbors.push(position);
    std::vector<std::vector<bool>> visited(map.heightInCells(), std::vector<bool>(map.widthInCells(), false));
    visited[position.y][position.x] = true;
    std::cout << "looking for nearest free space\n" << std::endl;
    while (!neighbors.empty()) { // bfs to look for nearest open cell
        Point<int> cell = neighbors.front();
        neighbors.pop();
        if (visited[cell.y][cell.x]) {
            continue;
        }

        visited[cell.y][cell.x] = true;
        if (abs(cell.x - original_position.x) > limit || abs(cell.y - original_position.y) > limit) { // no free cell near the desired goal
            continue;
        }

        Point<double> pose = grid_position_to_global_position(cell, map); // convert to robot pose
        pose_xyt_t goal;
        goal.x = pose.x;
        goal.y = pose.y;

        if (planner.isValidGoal(goal)) { // if current cell is a valid goal
            return cell;
        }

        // add the eight neighboring cells
        if (map.isCellInGrid(cell.x, cell.y - 1)) { // South
            neighbors.push(Point<int>(cell.x, cell.y - 1));
        }
        if (map.isCellInGrid(cell.x, cell.y + 1)) { // North
            neighbors.push(Point<int>(cell.x, cell.y + 1));
        }
        if (map.isCellInGrid(cell.x + 1, cell.y)) { // East
            neighbors.push(Point<int>(cell.x + 1, cell.y));
        }
        if (map.isCellInGrid(cell.x - 1, cell.y)) { // West
            neighbors.push(Point<int>(cell.x - 1, cell.y));
        }
        if (map.isCellInGrid(cell.x - 1, cell.y + 1)) { // North-West
            neighbors.push(Point<int>(cell.x - 1, cell.y + 1));
        }
        if (map.isCellInGrid(cell.x + 1, cell.y + 1)) { // North-East
            neighbors.push(Point<int>(cell.x + 1, cell.y + 1));
        }
        if (map.isCellInGrid(cell.x - 1, cell.y - 1)) { // South-West
            neighbors.push(Point<int>(cell.x - 1, cell.y - 1));
        }
        if (map.isCellInGrid(cell.x + 1, cell.y - 1)) { // South-East
            neighbors.push(Point<int>(cell.x + 1, cell.y - 1));
        }
    }
    return Point<int> (-1, -1); // cannot find any free cell
}

// TONY: takes in a path and returns its length
double path_length(const robot_path_t& path) {
    return path.path_length;
}

// TONY: max comparator for a min heap priority queue storing possible paths
// Robot should choose the shortest path
// Max comparator  p1 > p2
struct PathComparator {
    bool operator () (const robot_path_t & p1, const robot_path_t &p2) const {
        if (path_length(p1) > 0.2 && path_length(p2) <= 0.2) {
            return true; // p1 is better
        } else if (path_length(p1) <= 0.2 && path_length(p2) > 0.2) {
            return false; // p2 is better
        } else {
            return path_length(p1) > path_length(p2); // true if p2 is better
        }
    }
};


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// DONE: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    robot_path_t emptyPath;
    emptyPath.utime = robotPose.utime;
    emptyPath.path_length = 1;
    emptyPath.path.push_back(robotPose);

    // If no frontiers exist return path of length 1 with robot pose in it
    if (frontiers.empty()) {
        // std::cout << "frontier is empty\n";
        return emptyPath;
    }

    // push all possible cells into a set
    std::set<Point<int>> goals;
    for (frontier_t f : frontiers) { // iterate through all the frontiers
        for (Point<float> p : f.cells) {    // search for path for each cell in cell frontier?
            goals.insert(global_position_to_grid_cell(p, map));
        }
    }

    std::priority_queue<robot_path_t, std::vector<robot_path_t>, PathComparator> path_list;
    // iterate through the set of all possible points
    for (std::set<Point<int>>::iterator it = goals.begin(); it != goals.end(); ++it) {
        // store all the feasible paths in a priority queue sorted by distance
        pose_xyt_t goal;
        Point<double> pose = grid_position_to_global_position(*it, map);
        goal.x = pose.x;
        goal.y = pose.y;
        robot_path_t path = planner.planPath(robotPose, goal);
        if (path.path_length > 1) {
            path_list.push(path);
            std::cout << "pushed a frontier path\n";
        }
    }

    // if no path found, make another set to include nearby cells to the frontier
    if (path_list.empty()) {
        // std::cout << "found no path in frontiers???\n";
        std::set<Point<int>> neighbors;
        // std::set<Point<int>> new_goals;

        // iterate through each cell in previous set and add neighboring free cells
        for (std::set<Point<int>>::iterator it = goals.begin(); it != goals.end(); ++it) {
            Point<int> free_cell = search_to_nearest_free_space(*it, map, planner);
            if (free_cell.x != -1) { // if found a nearby free cell
                neighbors.insert(free_cell);
            }
        }
        
        // sort the two sets
        // std::stable_sort(goals.begin(), goals.end(), ComparePoint());
        // std::stable_sort(neighbors.begin(), neighbors.end(), ComparePoint());
        // get the set difference between origin set and the neighbors
        // std::set<Point<int>>::iterator it = std::set_difference(neighbors.begin(), neighbors.end(), goals.begin(), goals.end(), new_goals.begin(), ComparePoint());

        // iterate through the set of all possible points
        for (std::set<Point<int>>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
            // store all the feasible paths in a priority queue sorted by distance
            pose_xyt_t goal;
            Point<double> pose = grid_position_to_global_position(Point<int>(it->x, it->y), map);
            goal.x = pose.x;
            goal.y = pose.y;
            robot_path_t path = planner.planPath(robotPose, goal);
            if (path.path_length > 1) {
                std::cout << "pushed a neighboring cell path\n";
                path_list.push(path);
            }
        }
    }

    // if no path is found even in surrounding cells
    if (path_list.empty()) {
        std::cout << "still no path even in neighboring cells\n";
        return emptyPath;
    }

    robot_path_t shortest_path = path_list.top();
    // std::cout << "shortest path length: " << shortest_path.path_length << std::endl;
    return shortest_path;
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}