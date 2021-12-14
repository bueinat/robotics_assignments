#include "krembot.ino.h"
#include <cstdio>
#include <fstream>

// constants initialization
const int NMILESTONES = 100;
const int K = 5;
int moving_time = 1000;

// flags
bool is_close = false;
bool flag = true;

int col, row;
int **occupancyGrid;
int **coarseGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;

CVector2 dest(-2, -2);
CVector2 next_ml;
int next_ml_index = 1;
CVector2 toWalk;
CDegrees angle;
double distance_left;
SandTimer sandTimer;
Real coarseResolution;

// data structures
std::map<std::pair<float, float>, CVector2> milestones;
map<int, vector<float>> int_to_nodes_map;
map<int, vector<float>> nodes_to_int_map;
std::vector<CVector2> path;

enum State
{
    move,
    turn,
    whats_next,
    stop
} state;

int find_key_by_value(std::map<int, CoordPoint> map, CoordPoint value)
{
    int key = 0;
    for (auto &i : map)
        if (i.second == value)
            return i.first;
    return -1;
}

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
    int relativeResolution = (int)(robotSize / resolution / 4);     // TODO: divide by 4

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
    KdNodeVector nodes;
    std::vector<std::vector<float>> points;
    PRM_controller::fill_milestones_set(&milestones, &nodes, &points, height, width, NMILESTONES, coarseGrid);
    PRM_controller::write_grid_with_milestones("/home/einat/grid_with_ml.txt", coarseGrid, height, width);
    
    for (auto &a : points) {
        std::cout << a[0] << ", " << a[1] << std::endl;
    }

    // create graph with all points, edges are determined by knn
    Graph graph(NMILESTONES + 2);
    KdTree tree(&nodes);
    fill_graph(&tree, &graph, K);

    // insert start and end points to the tree
    // TODO: if one of those points is taken, print there's no path and return
    // float sx = 1 - origin.GetX();
    // float sy = 1 - origin.GetY();
    // float tx = -2 - origin.GetX();
    // float ty = -2 - origin.GetY();
    // KdNode start_node(std::vector<float>({sx, sy}));
    // KdNode end_node(std::vector<float>({tx, ty}));
    KdNode start_node(std::vector<float>({1, 1}));
    KdNode end_node(std::vector<float>({-2, -2}));
    int_to_nodes_map.insert({NMILESTONES, start_node.point});
    int_to_nodes_map.insert({NMILESTONES + 1, end_node.point});
    nodes.push_back(start_node); // starting point
    nodes.push_back(end_node);   // ending point
    KdTree tree_with_st(&nodes);
    insert_point_to_graph(start_node.point, &tree_with_st, &graph, K, NMILESTONES);
    insert_point_to_graph(end_node.point, &tree_with_st, &graph, K, NMILESTONES + 1);

    // find the path using BFS
    // the function I use gives the path in reverse order
    // so I switch the source and destination
    auto vec = graph.shortest_path_with_BFS(NMILESTONES + 1, NMILESTONES);

    // path in int to points
    for (int i = 0; i < vec.size(); i++)
    {
        CoordPoint p = int_to_nodes_map.at(vec[i]);
        path.push_back({p.at(0), p.at(1)});
        std::cout << "(" << p.at(0) << ", " << p.at(1) << ")" << std::endl;
    }

    // set first pair of points in the path
    next_ml = path.at(next_ml_index++);
    LOGERR << "first point: " << next_ml << std::endl;
    std::cout << "setup done! set size: " << milestones.size() << std::endl;
}

void PRM_controller::insert_point_to_graph(CoordPoint point, KdTree *tree, Graph *g, int k, int g_index)
{
    KdNodeVector result;
    tree->k_nearest_neighbors(point, k, &result);
    for (int b = 0; b < k; b++)
    {
        // find the keys matching the edge
        int d = find_key_by_value(int_to_nodes_map, result.at(b).point);
        if (d == -1)
        {
            LOGERR << "reached end" << std::endl;
            return;
        }
        // find out if there's a path
        CVector2 start = CVector2(point.at(0), point.at(1));
        CVector2 end = CVector2(result.at(b).point.at(0), result.at(b).point.at(1));
        if (is_path_clear(start, end, coarseGrid, true))
        {
            g->addEdge(g_index, d);
            std::cout << start << " -> " << end << std::endl;
        }
    }
}

void PRM_controller::fill_graph(KdTree *tree, Graph *g, int k)
{
    KdNodeVector result;
    for (int l = 0; l < NMILESTONES; l++)
    {
        tree->k_nearest_neighbors(int_to_nodes_map.at(l), k, &result);
        for (int b = 0; b < k; b++)
        {
            // find the keys matching the edge
            int d = find_key_by_value(int_to_nodes_map, result.at(b).point);
            if (d == -1)
                LOGERR << "reached end" << std::endl;
            // find out if there's a path
            CVector2 start = CVector2(int_to_nodes_map.at(l).at(0), int_to_nodes_map.at(l).at(1));
            CVector2 end = CVector2(result.at(b).point.at(0), result.at(b).point.at(1));
            if (is_path_clear(start, end, coarseGrid, true))
            {
                g->addEdge(l, d);
                std::cout << start << " -> " << end << std::endl;
            }
        }
    }
}

void PRM_controller::loop()
{
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    float y_in_grid = (pos.GetX() - origin.GetX()) / resolution;
    float x_in_grid = (pos.GetY() - origin.GetY()) / resolution;
    if (coarseGrid[(int)x_in_grid][(int)y_in_grid] == 1)
        LOGERR << "current point " << pos << "is occupied" << std::endl;

    std::cout << "the robot is at " << pos << std::endl;
    // in the first iteration, print out the robot's location on the map
    if (flag)
    {
        PRM_controller::write_grid_with_robot_location_and_destination_point("grid_with_st_init.txt", occupancyGrid, height, width);
        flag = false;
    }
    // find the vector to walk on
    // and extract the length and the angle of it.
    toWalk = next_ml - pos;
    angle = ToDegrees(toWalk.Angle());
    CDegrees angleDiff = NormalizedDifference(degreeX, angle);
    distance_left = toWalk.Length();
    // act based on the current state
    switch (state)
    {
    case State::turn:
    {
        std::cout << " with state turn, angle diff" << angleDiff.GetValue() << std::endl;
        int turning_orientation = -angleDiff.GetAbsoluteValue() / angleDiff.GetValue();
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
                    // std::cout << "finished" << std::endl;
                    state = State::whats_next;
                }
                else
                {
                    krembot.Base.drive(10, 0);
                }
            }
            // in case you're not close, drive in changing speeds according to your distance
            else if (distance_left > 0.3)
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
    case State::whats_next:
    {
        std::cout << "next point" << std::endl;

        if (next_ml == dest)
            state = State::stop;
        else {
            next_ml = path.at(next_ml_index++);
            state = State::turn;
            LOGERR << "next point: " << next_ml << std::endl;
        }
        break;
    }
    case State::stop:
    {
        krembot.Led.write(0, 255, 0);
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
                int pCol = (it->first.first - origin.GetX()) / resolution;
                int pRow = (it->first.second - origin.GetY()) / resolution;
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

// generate random float in range [min, max]
// taken from here: https://stackoverflow.com/questions/686353/random-float-number-generation
static float random_float(float min, float max)
{
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

void PRM_controller::fill_milestones_set(std::map<std::pair<float, float>, CVector2> *milestones, KdNodeVector *nodes, std::vector<std::vector<float>> *points,
                                         int height, int width, int nmilestones, int **grid)
{
    int i = 0;
    std::pair<float, float> new_key;
    std::vector<float> point(2);
    while (milestones->size() < nmilestones) // fill map until it has nmilestones elements
    {
        generate_random_point(width, height, grid, new_key); // fill the key with a new value
        if (milestones->find(new_key) == milestones->end())
        { // if the key doesn't exist already, insert it into the map
            milestones->insert(make_pair(new_key, CVector2(new_key.first, new_key.second)));
            point[0] = new_key.first;
            point[1] = new_key.second;
            int_to_nodes_map.insert({i++, point});
            nodes->push_back(KdNode(point));
            points->push_back(point);
        }
    }
}

void PRM_controller::generate_random_point(int width, int height, int **grid, std::pair<float, float> &oPair)
{
    float xmin = origin.GetX();
    float xmax = width * resolution + origin.GetX();
    float x = random_float(xmin, xmax);
    float y = random_float(origin.GetY(), height * resolution + origin.GetY());
    while (is_point_occupied(x, y, grid) == 1) // if (x, y) is occupied in the grid, generate new point
    {
        x = random_float(origin.GetX(), width * resolution + origin.GetX());
        y = random_float(origin.GetY(), height * resolution + origin.GetY());
    }

    //  fill the passed pair with the (x, y) values
    oPair.first = x;
    oPair.second = y;
}

int PRM_controller::is_point_occupied(float x, float y, int **grid)
{
    // transform (x, y) coordinates into grid cell
    float y_in_grid = (x - origin.GetX()) / resolution;
    float x_in_grid = (y - origin.GetY()) / resolution;
    return grid[(int)x_in_grid][(int)y_in_grid];
}

bool PRM_controller::is_path_clear(CVector2 startpoint, CVector2 endpoint, int **grid, bool add_origin)
{
    // if the points are really close, they're basically the same point
    // TODO: this is not accuate, maybe I should fix it
    if (Distance(startpoint, endpoint) < 1e-10)
        return true;
    // create a middle point which is exactly halfway between startpoint and endpoint
    CVector2 midpoint((startpoint.GetX() + endpoint.GetX()) / 2, (startpoint.GetY() + endpoint.GetY()) / 2);

    // if the mid-pointis occupied, the path is not clear
    if (is_point_occupied(midpoint.GetX(), midpoint.GetY(), grid))
        return false;

    bool flag = true;
    // transform each point into a grid cell
    int start_x = (startpoint.GetX() - origin.GetX()) / resolution;
    int start_y = (startpoint.GetY() - origin.GetY()) / resolution;
    int mid_x = (midpoint.GetX() - origin.GetX()) / resolution;
    int mid_y = (midpoint.GetY() - origin.GetY()) / resolution;
    int end_x = (endpoint.GetX() - origin.GetX()) / resolution;
    int end_y = (endpoint.GetY() - origin.GetY()) / resolution;

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
        flag = flag && is_path_clear(startpoint, midpoint, grid, add_origin);
    if (((end_x != mid_x) || (end_y != mid_y)) && (flag == true))
        flag = flag && is_path_clear(midpoint, endpoint, grid, add_origin);
    return flag;
}

void PRM_controller::print_nodes(const KdNodeVector &nodes)
{
    size_t i, j;
    for (i = 0; i < nodes.size(); ++i)
    {
        if (i > 0)
            cout << " ";
        cout << "(";
        for (j = 0; j < nodes[i].point.size(); j++)
        {
            if (j > 0)
                cout << ",";
            cout << nodes[i].point[j];
        }
        cout << ")";
    }
    cout << endl;
}
