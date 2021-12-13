#include "krembot.ino.h"
#include <cstdio>
#include <fstream>
// #include "BFS.h"

// given initializations
int col, row;
int **occupancyGrid;
int **coarseGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;
CVector2 dest(-2, -2);
CVector2 toWalk;
CDegrees angle;
double distance_left;
bool flag = true;
SandTimer sandTimer;
int moving_time = 1000;
bool is_close = false;

// my initializations
int NMILESTONES = 20;
Real coarseResolution;
std::map<std::pair<float, float>, CVector2> milestones;

enum State
{
    move,
    turn
} state;

void PRM_controller::setup()
{
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    state = turn;
    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

    // relative resolution indicates the number of cells which come into one in the coarse grid
    int relativeResolution = (int)(robotSize / resolution);

    // create coarse grid array and fill it with grid values
    coarseGrid = new int *[height]; // row = height
    for (int i = 0; i < height; i++)
        coarseGrid[i] = new int[width]; // col = width
    // re-size obstacles in grid
    PRM_controller::thickening_grid(occupancyGrid, coarseGrid, height, width, relativeResolution);

    // write both grids to files
    PRM_controller::write_grid("grid.txt", occupancyGrid, height, width);
    PRM_controller::write_grid("coarse_grid.txt", coarseGrid, height, width);

    // fill milestones with points, and write to file
    PRM_controller::fill_milestones_set(&milestones, height, width, NMILESTONES, coarseGrid);
    PRM_controller::write_grid_with_milestones("grid_with_ml.txt", coarseGrid, height, width);

    // // this is a test: iterate over all milestones and determine whether there's a path between them
    // std::map<std::pair<float, float>, CVector2>::iterator it_in;
    // std::map<std::pair<float, float>, CVector2>::iterator it_out;
    // for (it_out = milestones.begin(); it_out != milestones.end(); it_out++)
    //     for (it_in = milestones.begin(); it_in != milestones.end(); it_in++)
    //         std::cout << it_out->second << ", " << it_in->second << ": " << is_path_clear(it_out->second, it_in->second, coarseGrid) << std::endl;

    std::cout << "setup done! set size: " << milestones.size() << std::endl;
    // Graph::run_check();
}

void PRM_controller::loop()
{
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    std::cout << "the robot is at " << pos;
    // in the first iteration, print out the robot's location on the map
    if (flag)
    {
        PRM_controller::write_grid_with_robot_location_and_destination_point("grid_with_st_init.txt", occupancyGrid, height, width);
        flag = false;
    }
    // find the vector to walk on
    // and extract the length and the angle of it.
    toWalk = dest - pos;
    angle = ToDegrees(toWalk.Angle());
    CDegrees angleDiff = NormalizedDifference(degreeX, angle);
    distance_left = toWalk.Length();
    // act based on the current state
    switch (state)
    {
    case State::turn:
    {   
        std::cout << " with state turn, angle diff" << angleDiff.GetValue() << std::endl;
        int turning_orientation = - angleDiff.GetAbsoluteValue() / angleDiff.GetValue();
        // if you got more than 1 degree, turn towards your destination
        if (angleDiff.GetAbsoluteValue() > 100)
            krembot.Base.drive(0, turning_orientation * 100);
        else if (angleDiff.GetAbsoluteValue() > 1)
        {
            double turning_speed = angleDiff.GetAbsoluteValue() * 200 / 180.;
            krembot.Base.drive(0, turning_orientation * std::min(turning_speed, 100.));
        }
        // else, change mode to move
        else
        {
            state = State::move;
            sandTimer.start(moving_time);
            krembot.Led.write(0, 255, 0);
        }
        break;
    }
    // TODO: make it more neat and delete printing out
    case State::move:
    {
        std::cout << " with state move, distance " << distance_left << std::endl;
        // if time is up, change state to turn and set the distance for the following step
        if (sandTimer.finished())
        {    
            state = State::turn;
            krembot.Led.write(255, 0, 0);
        }
        else
        {
            // if you're close to the target, either say you've arrived or drive much slower
            if (is_close)
            {
                if (distance_left < 0.01)
                {
                    krembot.Base.stop();
                    std::cout << "finished" << std::endl;
                }
                else
                {
                    krembot.Base.drive(10, 0);
                }
            }
            // in case you're not close, drive in changing speeds according to your distance
            else if (distance_left > 0.5)
                krembot.Base.drive(100, 0);
            else if (distance_left > 0.1)
                krembot.Base.drive(50, 0);
            else
            {
                std::cout << "close!" << std::endl;
                is_close = true;
                PRM_controller::write_grid_with_robot_location_and_destination_point("grid_with_st.txt", occupancyGrid, height, width);
                krembot.Led.write(0, 0, 255);
            }
        }
        break;
    }
    }
}

void PRM_controller::write_grid(std::string filename, int **grid, int height, int width)
{
    // open file
    std::ofstream fid;
    fid.open(filename, std::ios_base::trunc);

    // write data to file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
            fid << grid[col][row];
        fid << std::endl;
    }

    fid.close();
}

void PRM_controller::write_grid_with_milestones(std::string filename, int **grid, int height, int width)
{
    // open file
    std::ofstream fid;
    fid.open(filename, std::ios_base::trunc);
    int symbol;

    // write data to file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
        {
            // symbol from grid
            symbol = grid[col][row];
            // find out if the cell has a point in it
            std::map<std::pair<float, float>, CVector2>::iterator it;
            for (it = milestones.begin(); it != milestones.end(); it++)
            {
                int pCol = it->first.first / resolution;
                int pRow = it->first.second / resolution;
                if ((col == pCol) && (row == pRow))
                {
                    symbol = 2;
                    break;
                }
            }
            fid << symbol;
        }
        fid << std::endl;
    }

    fid.close();
}

void PRM_controller::write_grid_with_robot_location_and_destination_point(std::string filename, int **grid, int height, int width)
{
    // open file
    std::ofstream fid;
    fid.open(filename, std::ios_base::trunc);
    int symbol;

    // write data to file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
        {
            // symbol from grid
            symbol = grid[col][row];
            int pCol = (pos.GetX() - origin.GetX()) / resolution;
            int pRow = (pos.GetY() - origin.GetY()) / resolution;
            if ((col == pCol) && (row == pRow))
                symbol = 2;
            pCol = (dest.GetX() - origin.GetX()) / resolution;
            pRow = (dest.GetY() - origin.GetY()) / resolution;
            if ((col == pCol) && (row == pRow))
                symbol = 3;
            fid << symbol;
        }
        fid << std::endl;
    }

    fid.close();
}

void PRM_controller::thickening_grid(int **origGrid, int **newGrid, int height, int width, Real res)
{
    // by default: fill with 0 = free
    int filling_value = 0;
    // iterate over grid cells
    for (int i = 0; i < height; i += res)
    {
        for (int j = 0; j < width; j += res)
        {
            // iterate over coarseGrid cells and find out for each if it's occupied or not
            // note that the size of both grid (and cells in them) is the sanw
            filling_value = 0;
            for (int kheight = 0; kheight < res; kheight++)
            {
                for (int kwidth = 0; kwidth < res; kwidth++)
                {
                    // if any cell in the big cell is occupied, the whole cell would be occupied
                    if (origGrid[i + kheight][j + kwidth] == 1)
                    {
                        filling_value = 1;
                        break;
                    }
                }
                if (filling_value == 1)
                    break;
            }

            // fill in the value in the tested cell above
            for (int kheight = 0; kheight < res; kheight++)
                for (int kwidth = 0; kwidth < res; kwidth++)
                    newGrid[i + kheight][j + kwidth] = filling_value;
        }
    }
}

// generate random float in range [0-max]
static float random_float(float max)
{
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX / max);
}

void PRM_controller::fill_milestones_set(std::map<std::pair<float, float>, CVector2> *milestones,
                                         int height, int width, int nmilestones, int **grid)
{
    std::pair<float, float> new_key;
    while (milestones->size() < nmilestones) // fill map until it has nmilestones elements
    {
        generate_random_point(width, height, grid, new_key); // fill the key with a new value
        if (milestones->find(new_key) == milestones->end())  // if the key doesn't exist already, insert it into the map
            milestones->insert(make_pair(new_key, CVector2(new_key.first, new_key.second)));
    }
}

void PRM_controller::generate_random_point(int width, int height, int **grid, std::pair<float, float> &oPair)
{
    float x = random_float(width * resolution);
    float y = random_float(height * resolution);
    while (is_point_occupied(x, y, grid) == 1) // if (x, y) is occupied in the grid, generate new point
    {
        x = random_float(width * resolution);
        y = random_float(height * resolution);
    }

    //  fill the passed pair with the (x, y) values
    oPair.first = x;
    oPair.second = y;
}

int PRM_controller::is_point_occupied(float x, float y, int **grid)
{
    // transform (x, y) coordinates into grid cell
    float x_in_grid = x / resolution;
    float y_in_grid = y / resolution;
    return grid[(int)x_in_grid][(int)y_in_grid];
}

bool PRM_controller::is_path_clear(CVector2 startpoint, CVector2 endpoint, int **grid)
{
    // create a middle point which is exactly halfway between startpoint and endpoint
    CVector2 midpoint((startpoint.GetX() + endpoint.GetX()) / 2, (startpoint.GetY() + endpoint.GetY()) / 2);

    // if the mid-pointis occupied, the path is not clear
    if (is_point_occupied(midpoint.GetX(), midpoint.GetY(), grid))
        return false;

    bool flag = true;
    // transform each point into a grid cell
    int start_x = startpoint.GetX() / resolution;
    int start_y = startpoint.GetY() / resolution;
    int mid_x = midpoint.GetX() / resolution;
    int mid_y = midpoint.GetY() / resolution;
    int end_x = endpoint.GetX() / resolution;
    int end_y = endpoint.GetY() / resolution;

    // if the start point cell and the end point cell are identical, a path exists
    if ((start_x == end_x) && (start_y == end_y))
        return true;

    // if the mid cell is occupied, there is no path
    if (grid[mid_x][mid_y] == 1)
        return false;

    // if the start point and the end point are adjacent, check if they're both free
    // and return accordingly
    if (((start_x + 1 == end_x) || (start_x - 1 == end_x)) && (start_y == end_y))
    {
        if ((grid[start_x][start_y] == 0 && (grid[end_x][end_y] == 0)))
            return true;
        return false;
    }
    if ((start_x == end_x) && ((start_y + 1 == end_y) || (start_y - 1 == end_y)))
    {
        if ((grid[start_x][start_y] == 0 && (grid[end_x][end_y] == 0)))
            return true;
        return false;
    }

    // If you got so far, run the same function on (start, mid) and (mid, end)
    // assuming none of them are identical
    if (((start_x != mid_x) || (start_y != mid_y)) && (flag == true))
        flag = flag && is_path_clear(startpoint, midpoint, grid);
    if (((end_x != mid_x) || (end_y != mid_y)) && (flag == true))
        flag = flag && is_path_clear(midpoint, endpoint, grid);
    return flag;
}
