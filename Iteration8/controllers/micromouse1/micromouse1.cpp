// File:          maze_controller.cpp
// Date:          10-Oct-20
// Description:
// Author:        Priyanka Prakash Chand

// Modifications:
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <stack>
#include <queue>
#include <unordered_map>

#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>

#include <ctime>

#include "../common/MazeBot.hpp"

#include "Utility/MazeGraph.h"
#include "Utility/FloodFillMapReader.h"
#include "Utility/Maze.h"
#include "Utility/Path.h"
#include "Utility/IMazeReader.h"
#include "Utility/Vertex.h"

using namespace webots;

// environment variables
int numCellsExplored = 0, numNewCellsExplored = 0, numUpdates = 0, numNewUpdates = 0;
int numTurnsTaken = 0, numNewTurnsTaken = 0;
int mazeSize = 16; // 16X16
int centerX = mazeSize / 2;
int centerY = mazeSize / 2;
int startX = 0;
int startY = 0;
int EXPLORED_GOAL_X, EXPLORED_GOAL_Y;
MazeBot *mazeBot;
Supervisor *supervisor;
Node *robot_node;
int width = 8;
int height = 8;

/*
    * orientation-stores the direction faced by the arrow head
    *  possible orientation values:
    *       N=> North
    *       S=>South
    *       W=>West
    *       E=>East
    * 
    */
// Environment Variables
char orientation = 'N'; // current orientation
enum mode
{
    FIND_CENTRE,
    FIND_START
};
mode searchMode = FIND_CENTRE;

int x = 0,
    y = 0; // current coordinates

// priority queue to design a min heap-Data structure that gives priority to the minumum
template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>>
        elements;

    inline bool empty() const
    {
        return elements.empty();
    }

    inline void put(T item, priority_t priority)
    {
        elements.emplace(priority, item);
    }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

//---------------------function signatures--------------------
void fixBoundaries(vector<vector<Cell>> &map, int X, int Y);
void log(const std::string &text);
void logInline(const std::string &text);
const std::string currentDateTime();
bool isGoal(int x, int y);
bool matchGoal(Coordinates current, Coordinates goal);
bool isSafe(int X, int Y);
bool isStart(int X, int Y);
void updateposition(char currentMove);
void exploreCell(vector<vector<Cell>> &map, int x, int y);
void initializeMaze(vector<vector<Cell>> &map);
void findGoal(vector<vector<Cell>> &map);                          // hardcoded for start and goal
void findStartToGoal(vector<vector<Cell>> &map, Coordinates goal); // can find any goal from any start position
Cell &getFrontCell(vector<vector<Cell>> &map);
Cell &getLeftCell(vector<vector<Cell>> &map);
Cell &getRightCell(vector<vector<Cell>> &map);
void getMinDistanceDirection(vector<vector<Cell>> &map, int &minDistance, char &minDirection);
void floodOpenNeighbours(vector<vector<Cell>> &map, Coordinates goal);
void moveInDirection(vector<vector<Cell>> &map, char direction);
int countDigit(int n);
void print_maze(vector<vector<Cell>> &map);
void estimateCosts(vector<vector<Cell>> &map, Coordinates goal);
void resetToStart(); //move back to 00 and make orientation =north
void moveNstepsinDirection(vector<vector<Cell>> &map, Coordinates current, Coordinates next);
void defineMaze(Maze &maze, vector<vector<Cell>> &map, Coordinates goal);
void getAstarPath(vector<vector<Cell>> &map, Coordinates goal);

//-----------------------------Search algorithm function headers----------------------

void BFSEarlyStop(MazeGraph &graph, Coordinates start);
inline double heuristic(Coordinates a, Coordinates b);
void dijkstra_search(MazeGraph &graph,
                     vector<vector<Cell>> &map,
                     Coordinates start,
                     Coordinates goal,
                     std::unordered_map<Coordinates, Coordinates, HashFunction> &came_from,
                     std::unordered_map<Coordinates, double, HashFunction> &cost_so_far, std::stack<Coordinates> &path);

void aStar_search(MazeGraph &graph,
                  vector<vector<Cell>> &map,
                  Coordinates start,
                  Coordinates goal,
                  std::unordered_map<Coordinates, Coordinates, HashFunction> &came_from,
                  std::unordered_map<Coordinates, double, HashFunction> &cost_so_far, std::stack<Coordinates> &path);

void moveToNeighbouringCoordinates(vector<vector<Cell>> &map, int targetX, int targetY);
void followpath(vector<vector<Cell>> &map, std::stack<Coordinates> &path);
//----------------------------------------------main()--------------------------------

// entry point of the controller
int main(int argc, char **argv)
{
    // create a mazeBot instance
    Robot *robot = new Robot();
    mazeBot = new MazeBot(robot);
    Supervisor *supervisor = (Supervisor *)robot;
    robot_node = supervisor->getFromDef("JEREMY");

    std::cout << "Resetting to Start\n";
    resetToStart();
    std::cout << "Reset done \n";
    vector<vector<Cell>> map(width, vector<Cell>(height));
    std::cout << "Iniitalizing maze\n";
    initializeMaze(map);
    log("Start Time = " + currentDateTime());
    Coordinates goal = Coordinates{3, 3};
    estimateCosts(map, goal);
    std::cout << "Starting flood fill\n";
    findStartToGoal(map, goal);
    // get all unexplored cells
    queue<Coordinates> unexploredCells;
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {

            if (!map[i][j].getHasBeenExplored())
            {

                unexploredCells.push(Coordinates{i, j});
            }
        }
    }
    // generate complete map while (!unexploredCells.empty())
    while (!unexploredCells.empty())
    {
        Coordinates goal = unexploredCells.front();
        unexploredCells.pop();
        if (!map[goal.x][goal.y].getHasBeenExplored())
        {
            estimateCosts(map, goal);
            std::cout << "Moving to Goal Location: [" << goal.x << " " << goal.y << "]\n";
            findStartToGoal(map, goal);
        }
    }
    // std::cout << "Moving 1 step forward\n";
    // mazeBot->forward(1);
    // std::cout << "turning right\n";
    // mazeBot->right();
    // std::cout << "Moving 1 step forward\n";
    // mazeBot->forward(1);

    log("End Time = " + currentDateTime());
    log("Complete maze has been explored");
    resetToStart();
    log("Converting to Graph");
    MazeGraph graph(map, width, height);
    log("printing  Graph");
    graph.printGraph();
    //  data structures for a star
    std::unordered_map<Coordinates, Coordinates, HashFunction> came_from_aStar;
    std::unordered_map<Coordinates, double, HashFunction> cost_so_far_aStar;
    std::stack<Coordinates> path_aStar;
    srand((unsigned)time(0));
    for (int i = 0; i < 10; i++)
    {
        log("Test " + std::to_string(i) + "\n");
        Coordinates testGoal = Coordinates{(rand() % width), (rand() % height)};
        log("Test Goal:" + std::to_string(testGoal.x) + " ," + std::to_string(testGoal.y));
        log("A* Search on graph with infinitely high heuristic value for unexplored cells");
        resetToStart();
        log("a_star path planning:\nStart Time = " + currentDateTime());
        getAstarPath(map, testGoal);
        log("End Time = " + currentDateTime());
        came_from_aStar.clear();
        cost_so_far_aStar.clear();
    }
    delete mazeBot;
    return 0; //EXIT_SUCCESS
}

//---------------------- function defintions-------------------------------
void log(const std::string &text)
{
    std::cerr << text << std::endl;
}
bool matchGoal(Coordinates current, Coordinates goal)
{
    return (current.x == goal.x && current.y == goal.y);
}
bool isStart(int X, int Y)
{
    return (X == startX && Y == startY);
}
void resetToStart()
{
    double X = -0.36;
    double Y = 0;
    double Z = 0.35;

    Field *translationField = robot_node->getField("translation");

    double sfVec3f[] = {X, Y, Z};
    translationField->setSFVec3f(sfVec3f);

    // make sure orientation is correct
    Field *rotationField = robot_node->getField("rotation");

    double rotation[] = {0, 1, 0, 0};
    rotationField->setSFRotation(rotation);
    x = 0, y = 0;
    orientation = 'N';
}
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime()
{
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    printf("The current date/time is: %s", asctime(timeinfo));
    return asctime(timeinfo);
}

// robot has knowledge about the maze area so ensure that it is not being exceeded
bool isSafe(int X, int Y)
{
    if (X < 0 || X >= width)
        return false;
    if (Y < 0 || Y >= height)
        return false;
    return true;
}

// initializes the map datastructure
void initializeMaze(vector<vector<Cell>> &map)
{

    for (int i = 0; i < width; i++)
    {
        map[i].reserve(height);
    }
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            map[i][j].setcellAddress(Coordinates{i, j});
        }
    }
}
//set  flood fill costs assuming no walls between cells
void estimateCosts(vector<vector<Cell>> &map, Coordinates goal)
{
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {

            int X = std::abs(i - goal.x);

            int Y = std::abs(j - goal.y);
            int floodFillCost = X + Y;
            map[i][j].setFloodFillCost(floodFillCost);
        }
    }
}

void exploreCell(vector<vector<Cell>> &map, int x, int y)
{
    // if cell has already been explored before, then return back
    if (map[x][y].getHasBeenExplored())
    {
        return;
    }

    map[x][y].sethasBeenExplored(true);

    numCellsExplored++;
    bool wallFront = mazeBot->wallAhead();
    bool wallLeft = mazeBot->wallLeft();
    bool wallRight = mazeBot->wallRight();
    std::cout << "Exploring cell \n" + std::to_string(x) + " " + std ::to_string(y) + "- Facing" << orientation << "\n";
    std::cout << "wall Ahead= " << std::boolalpha << wallFront << "\n";
    std::cout << "wall left= " << std::boolalpha << wallLeft << "\n";
    std::cout << "wall Right= " << std::boolalpha << wallRight << "\n";

    switch (orientation)
    {
    case 'N':

        map[x][y].setNorthWall(wallFront);

        map[x][y].setEastWall(wallRight);
        map[x][y].setWestWall(wallLeft);

        break;
    case 'S':

        map[x][y].setSouthWall(wallFront);
        map[x][y].setEastWall(wallLeft);
        map[x][y].setWestWall(wallRight);
        break;
    case 'W':

        map[x][y].setWestWall(wallFront);
        map[x][y].setSouthWall(wallLeft);
        map[x][y].setNorthWall(wallRight);
        break;
    case 'E':

        map[x][y].setEastWall(wallFront);
        map[x][y].setNorthWall(wallLeft);
        map[x][y].setSouthWall(wallRight);
        break;
    default:
        int a = 0;
        a = a + 0;
    }
}

// uses current position and orientation to return coordinates of front cell --doesnt check if coordinates are within boundaries
Cell &getFrontCell(vector<vector<Cell>> &map)
{

    switch (orientation)
    {
    case 'N':
        return map[x][y + 1];
        break;
    case 'S':
        return map[x][y - 1];
        break;
    case 'E':
        return map[x + 1][y];
        break;
    case 'W':
        return map[x - 1][y];
        break;
    default:
        return map[x][y];
    }
}
// uses current position and orientation to return coordinates of left cell --doesnt check if coordinates are within boundaries
Cell &getLeftCell(vector<vector<Cell>> &map)
{

    switch (orientation)
    {
    case 'N':
        return map[x - 1][y];
        break;
    case 'S':
        return map[x + 1][y];
        break;
    case 'E':
        return map[x][y + 1];
        break;
    case 'W':
        return map[x][y - 1];
        break;
    default:
        return map[x][y];
        ;
    }
}

// uses current position and orientation to return coordinates of right cell --doesnt check if coordinates are within boundaries
Cell &getRightCell(vector<vector<Cell>> &map)
{

    switch (orientation)
    {
    case 'N':
        return map[x + 1][y];
        break;
    case 'S':
        return map[x - 1][y];
        break;
    case 'E':
        return map[x][y - 1];
        break;
    case 'W':
        return map[x][y + 1];
        break;
    default:
        return map[x][y];
    }
}

enum LocalWall
{
    FRONT,
    LEFT,
    RIGHT
};
//get min distance relative to current position and relative position f = front, b = behind, l=left,r=right
char getWall(LocalWall Face)
{
    switch (Face)
    {
    case FRONT:
        if (orientation == 'N')
            return 'N';
        else if (orientation == 'S')
            return 'S';
        else if (orientation == 'W')
            return 'W';
        else if (orientation == 'E')
            return 'E';
        else
            return 'b';

        break;
    case LEFT:
        if (orientation == 'N')
            return 'W';
        else if (orientation == 'S')
            return 'E';
        else if (orientation == 'W')
            return 'S';
        else if (orientation == 'E')
            return 'N';
        else
            return 'b';
        break;
    case RIGHT:
        if (orientation == 'N')
            return 'E';
        else if (orientation == 'S')
            return 'W';
        else if (orientation == 'W')
            return 'N';
        else if (orientation == 'E')
            return 'S';
        else
            return 'b';
        break;
    default:
        return 'b';
    }
}
void getMinDistanceDirection(vector<vector<Cell>> &map, int &minDistance, char &minDirection)
{
    LocalWall Face;
    // cell has been explored so simplly check walls- dont sense again;
    Face = FRONT;
    if (!map[x][y].getWallStatus(getWall(Face)))
    {
        Cell front = getFrontCell(map);

        if (minDistance > front.getFloodFillCost())
        {
            minDistance = front.getFloodFillCost();
            minDirection = 'f';
        }
    }
    Face = LEFT;
    if (!map[x][y].getWallStatus(getWall(Face)))
    {
        Cell left = getLeftCell(map);
        if (minDistance > left.getFloodFillCost())
        {
            minDistance = left.getFloodFillCost();
            minDirection = 'l';
        }
    }
    Face = RIGHT;
    if (!map[x][y].getWallStatus(getWall(Face)))
    {
        Cell right = getRightCell(map);
        if (minDistance > right.getFloodFillCost())
        {
            minDistance = right.getFloodFillCost();
            minDirection = 'r';
        }
    }
    if (map[x][y].getPrevVisitedCell() != nullptr)
    {
        Cell *back = map[x][y].getPrevVisitedCell();
        if (minDistance > back->getFloodFillCost())
        {
            minDistance = back->getFloodFillCost();
            minDirection = 'b';
        }
    }
}

void floodOpenNeighbours(vector<vector<Cell>> &map, Coordinates goal)
{

    std::stack<Coordinates> floodStack;
    int minDistance = mazeSize * 2;

    int cellX, cellY;
    floodStack.push(Coordinates({x, y}));
    while (!floodStack.empty())
    {
        cellX = floodStack.top().x;
        cellY = floodStack.top().y;

        floodStack.pop();
        if (matchGoal(Coordinates{cellX, cellY}, goal))
            continue;

        Cell cell = map[cellX][cellY];

        if (cell.getHasBeenExplored()) // for an explored sell distance is large when there is a wall between cells
        {

            minDistance = mazeSize * 2;
            int D1 = (!map[cellX][cellY].hasNorthWall()) ? map[cellX][cellY + 1].getFloodFillCost() : mazeSize * 2;
            int D2 = (!map[cellX][cellY].hasSouthWall()) ? map[cellX][cellY - 1].getFloodFillCost() : mazeSize * 2;
            int D3 = (!map[cellX][cellY].hasWestWall()) ? map[cellX - 1][cellY].getFloodFillCost() : mazeSize * 2;
            int D4 = (!map[cellX][cellY].hasEastWall()) ? map[cellX + 1][cellY].getFloodFillCost() : mazeSize * 2;
            minDistance = std::min(D1, D2);
            minDistance = std::min(minDistance, D3);
            minDistance = std::min(minDistance, D4);
            //----------add to stack

            if (map[cellX][cellY].getFloodFillCost() != 1 + minDistance)
            {
                map[cellX][cellY].setFloodFillCost(1 + minDistance);

                if (!map[cellX][cellY].hasNorthWall())
                {
                    floodStack.push(Coordinates{cellX, cellY + 1});
                }
                if (!map[cellX][cellY].hasSouthWall())
                {
                    floodStack.push(Coordinates{cellX, cellY - 1});
                }
                if (!map[cellX][cellY].hasWestWall())
                {
                    floodStack.push(Coordinates{cellX - 1, cellY});
                }
                if (!map[cellX][cellY].hasEastWall())
                {
                    floodStack.push(Coordinates{cellX + 1, cellY});
                }
            }
        }
        else
        {

            int minD = mazeSize * 2;
            // a cell that has not been explored has no walls so all neighbours are accessible

            int d1 = isSafe(cellX + 1, cellY) ? map[cellX + 1][cellY].getFloodFillCost() : mazeSize * 2;
            int d2 = isSafe(cellX - 1, cellY) ? map[cellX - 1][cellY].getFloodFillCost() : mazeSize * 2;
            int d3 = isSafe(cellX, cellY + 1) ? map[cellX][cellY + 1].getFloodFillCost() : mazeSize * 2;
            int d4 = isSafe(cellX, cellY - 1) ? map[cellX][cellY - 1].getFloodFillCost() : mazeSize * 2;
            minD = std::min(d1, d2);
            minD = std::min(minD, d3);
            minD = std::min(minD, d4);
            if (map[cellX][cellY].getFloodFillCost() != 1 + minD)
            {
                map[cellX][cellY].setFloodFillCost(1 + minD);

                if (isSafe(cellX + 1, cellY))
                {
                    floodStack.push(Coordinates{cellX + 1, cellY});
                }
                if (isSafe(cellX - 1, cellY))
                {
                    floodStack.push(Coordinates{cellX - 1, cellY});
                }
                if (isSafe(cellX, cellY + 1))
                {
                    floodStack.push(Coordinates{cellX, cellY + 1});
                }
                if (isSafe(cellX, cellY - 1))
                {
                    floodStack.push(Coordinates{cellX, cellY - 1});
                }
            }
        }
    }
}

// updates the orientation and current, x, y coordinates based on what move was made
// must be called after each move
void updateposition(char currentMove)
{
    // update the orientation and coordinates
    switch (orientation)
    {
    case 'N':
        if (currentMove == 'l')
            orientation = 'W';
        else if (currentMove == 'r')
            orientation = 'E';

        else if (currentMove == 'f')
            ++y;
        break;
    case 'S':
        if (currentMove == 'l')
            orientation = 'E';
        else if (currentMove == 'r')
            orientation = 'W';
        else if (currentMove == 'f')
            --y;
        break;
    case 'E':
        if (currentMove == 'l')
            orientation = 'N';
        else if (currentMove == 'r')
            orientation = 'S';
        else if (currentMove == 'f')
            ++x;
        break;
    case 'W':
        if (currentMove == 'l')
            orientation = 'S';
        else if (currentMove == 'r')
            orientation = 'N';
        else if (currentMove == 'f')
            --x;
        break;
    default:
        x = 0, y = 0;
    }
}

// move one step (one cell) in specified direction f-move front ,b- move back,r-move right, l-moveleft
void moveInDirection(vector<vector<Cell>> &map, char direction)
{
    // move to the  neighbouring cell with the lowest distance cost
    int prevX = x, prevY = y;
    if (direction == 'f')
    {
        // move forward by one cell
        mazeBot->forward(1);
        updateposition('f');
    }
    else if (direction == 'l')
    {
        // turn left in place
        mazeBot->left();
        updateposition('l');
        // move forward by one cell
        mazeBot->forward(1);
        updateposition('f');
        numTurnsTaken++;
    }

    else if (direction == 'r')
    {

        // turn right in place
        mazeBot->right();
        updateposition('r');
        // move forward by one cell
        mazeBot->forward(1);
        updateposition('f');
        numTurnsTaken++;
    }
    else if (direction == 'b')
    {
        // turn right in place
        mazeBot->right();
        updateposition('r');
        // turn right in place
        mazeBot->right();
        updateposition('r');
        // move forward by one cell
        mazeBot->forward(1);
        updateposition('f');
        numTurnsTaken++;
        numTurnsTaken++;
    }

    map[x][y].setPrevVisitedCell(&map[prevX][prevY]);
}

// use flood fill to find way from start to goal and simulataneosly update map
void findStartToGoal(vector<vector<Cell>> &map, Coordinates goal)
{

    bool destinationFound = false;
    int minDistance;
    char minDirection;

    while (!destinationFound)
    {

        exploreCell(map, x, y); // explore current cell
        fixBoundaries(map, x, y);
        minDistance = mazeSize * 5;
        // // if goal found-> exit while loop
        if (matchGoal(Coordinates{x, y}, goal))
        {
            destinationFound = true;
        }
        if (!destinationFound)
        {
            getMinDistanceDirection(map, minDistance, minDirection);
            // check if reflooding is required
            if ((map[x][y].getFloodFillCost() != 1 + minDistance))
            {
                //     // reflood
                floodOpenNeighbours(map, goal);

                // after reflooding get new min distance neighbours
                getMinDistanceDirection(map, minDistance, minDirection); //get neighbour with lowest distance
            }

            moveInDirection(map, minDirection);
            std::cout << "At Cell [" << x << " " << y << "]\n";
        }
    }
}

void dijkstra_search(MazeGraph &graph,
                     vector<vector<Cell>> &map,
                     Coordinates start,
                     Coordinates goal,
                     std::unordered_map<Coordinates, Coordinates, HashFunction> &came_from,
                     std::unordered_map<Coordinates, double, HashFunction> &cost_so_far, std::stack<Coordinates> &path)
{

    PriorityQueue<Coordinates, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;
    Coordinates G = {-1, -1};
    while (!frontier.empty())
    {
        // get
        Coordinates current = frontier.get();

        if (matchGoal(Coordinates{current.x, current.y}, goal))
        {
            G = current;
            break;
        }

        for (Coordinates next : graph.neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph.cost(map, current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;

                frontier.put(next, new_cost);
                came_from[next] = current;
            }
        }
    }
    // reconstrucnt path
    Coordinates current = G;
    while (!(current == start))
    {

        path.push(current);
        current = came_from[current];
    }

    path.push(current);
}

inline double heuristic(vector<vector<Cell>> &map, Coordinates current, Coordinates goal)
{
    if (map[current.x][current.y].getHasBeenExplored())
    { // manhattan
        int D = 1;
        return D * std::abs(current.x - goal.x) + std::abs(current.y - goal.y);
    }
    else
    {
        // return std::numeric_limits<double>::max();
        return 1000.90;
    }
}

void aStar_search(MazeGraph &graph,
                  vector<vector<Cell>> &map,
                  Coordinates start,
                  Coordinates goal,
                  std::unordered_map<Coordinates, Coordinates, HashFunction> &came_from,
                  std::unordered_map<Coordinates, double, HashFunction> &cost_so_far, std::stack<Coordinates> &path)
{

    PriorityQueue<Coordinates, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;
    Coordinates G = {-1, -1};
    while (!frontier.empty())
    {
        // get
        Coordinates current = frontier.get();

        if (matchGoal(Coordinates{current.x, current.y}, goal))
        {
            G = current;
            break;
        }

        for (Coordinates next : graph.neighbors(current))
        {

            double new_cost = cost_so_far[current] + graph.cost(map, current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(map, next, goal);

                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }

    // reconstrucnt path
    Coordinates current = G;
    std::cout << "Path:\n";
    while (!(current == start))
    {

        std::cout << "[" << current.x << " " << y << "]\n";
        path.push(current);
        current = came_from[current];
    }

    path.push(current);
}

// moves to neighbour updates orientation and x,y coordinates
void moveToNeighbouringCoordinates(vector<vector<Cell>> &map, int targetX, int targetY)
{
    switch (orientation)
    {

    case 'N':
        if (targetX == (x - 1))
            moveInDirection(map, 'l');
        else if (targetX == (x + 1))
            moveInDirection(map, 'r');
        else if (targetY == (y + 1))
            moveInDirection(map, 'f');
        else if (targetY == (y - 1))
            moveInDirection(map, 'b');
        break;

    case 'S':
        if (targetX == (x - 1))
            moveInDirection(map, 'r');
        else if (targetX == (x + 1))
            moveInDirection(map, 'l');
        else if (targetY == (y + 1))
            moveInDirection(map, 'b');
        else if (targetY == (y - 1))
            moveInDirection(map, 'f');
        break;

    case 'E':
        if (targetX == (x - 1))
            moveInDirection(map, 'b');
        else if (targetX == (x + 1))
            moveInDirection(map, 'f');
        else if (targetY == (y + 1))
            moveInDirection(map, 'l');
        else if (targetY == (y - 1))
            moveInDirection(map, 'r');
        break;

    case 'W':
        if (targetX == (x - 1))
            moveInDirection(map, 'f');
        else if (targetX == (x + 1))
            moveInDirection(map, 'b');
        else if (targetY == (y + 1))
            moveInDirection(map, 'r');
        else if (targetY == (y - 1))
            moveInDirection(map, 'l');
        break;

    default:
        return;
    }
}

void followpath(vector<vector<Cell>> &map, std::stack<Coordinates> &path)
{
    log("FollowPath:");
    numTurnsTaken = 0;
    log("Start Time= " + currentDateTime());
    // while stack not empty get next coordinate
    // move to next neighbour
    while (!path.empty())
    {
        moveToNeighbouringCoordinates(map, path.top().x, path.top().y);
        std::cout << "At Cell [" << x << " " << y << "]\n";
        path.pop();
    }

    log("End Time= " + currentDateTime());
    log("Number of turns taken: " + std::to_string(numTurnsTaken));
}
void fixBoundaries(vector<vector<Cell>> &map, int X, int Y)
{
    if (X == 0)
    {
        map[X][Y].setWestWall(true);
    }
    if (X == width - 1)
    {
        map[X][Y].setEastWall(true);
    }

    if (Y == 0)
    {
        map[X][Y].setSouthWall(true);
    }

    if (Y == height - 1)
    {
        map[X][Y].setNorthWall(true);
    }
}

void findPath(Maze &maze, Path &p)
{
    maze.reset();
    p.findPath(maze.maze_begin(), maze.maze_end());

    std::cout << "Path calculated " << std::endl;

    for (auto &x : p.getPath())
    {
        std::cout << '(' << x->getColumn() << ' ' << x->getRow() << ") " << std::endl;
    }
}

// created a floodfill map to maze

void defineMaze(Maze &maze, vector<vector<Cell>> &map, Coordinates goal)
{

    FloodFillMapReader floodFillMapReader(map, Coordinates{startX, startY}, goal, width, height);
    maze.setMaze(floodFillMapReader);
}

void getAstarPath(vector<vector<Cell>> &map, Coordinates goal)
{
    Maze maze;
    defineMaze(maze, map, goal);

    Path p;
    findPath(maze, p);

    std::list<Vertex *> path = p.getPath();
    log("FollowPath:");
    numTurnsTaken = 0;
    log("Start Time= " + currentDateTime());
    for (auto p : path)
    {
        std::cerr << "y= " << (height - p->getRow() - 1);
        std::cerr << "x= " << p->getColumn() << std::endl;
        // move to next neighbour
        // moveToNeighbouringCoordinates(map, path.top().x, path.top().y);
        moveNstepsinDirection(map, Coordinates{x, y}, Coordinates{p->getColumn(), height - p->getRow() - 1});
    }

    log("End Time= " + currentDateTime());
    log("Number of turns taken: " + std::to_string(numTurnsTaken));
}

void moveNstepsinDirection(vector<vector<Cell>> &map, Coordinates current, Coordinates next)
{
    // next should differ from current in one of the coordinates only
    if (current.x == next.x && current.y == next.y)
    {
        // can either move Forward or left or right not diagnal
        return;
    }

    if (current.x != next.x && current.y != next.y)
    {
        // can either move Forward or left or right not diagnal
        return;
    }

    int nsteps = 0;
    // if current is to the left of next, move right by n steps
    if (current.x < next.x)
    {
        // move one step right (turn right in place and move by one step)
        if (orientation == 'N')
        {
            moveInDirection(map, 'r');
        }
        else if (orientation == 'S')
        {
            moveInDirection(map, 'l');
        }
        else if (orientation == 'W')
        {
            moveInDirection(map, 'b');
        }
        else
        {
            // no need to turn
            moveInDirection(map, 'f');
        }
        nsteps = next.x - current.x - 1; // remove one step
    }

    // if current is to the right of next, move left by n steps
    else if (current.x > next.x)
    {

        // move one step right (turn right in place and move by one step)
        if (orientation == 'N')
        {
            moveInDirection(map, 'l');
        }
        else if (orientation == 'S')
        {
            moveInDirection(map, 'r');
        }
        else if (orientation == 'W')
        {
            moveInDirection(map, 'f');
        }
        else
        {

            moveInDirection(map, 'b');
        }
        nsteps = current.x - next.x - 1;
    }
    else if (current.y > next.y)
    {
        // move one step downwards
        if (orientation == 'N')
        {
            moveInDirection(map, 'b');
        }
        else if (orientation == 'S')
        {
            moveInDirection(map, 'f');
        }
        else if (orientation == 'W')
        {
            moveInDirection(map, 'l');
        }
        else
        {
            moveInDirection(map, 'r');
        }
        nsteps = current.y - next.y - 1;
    }
    else
    {
        // need to move forward
        // move one step right (turn right in place and move by one step)
        if (orientation == 'N')
        {
            moveInDirection(map, 'f');
        }
        else if (orientation == 'S')
        {
            moveInDirection(map, 'b');
        }
        else if (orientation == 'W')
        {
            moveInDirection(map, 'r');
        }
        else
        {
            // no need to turn
            moveInDirection(map, 'l');
        }
        nsteps = next.y - current.y - 1;
    }
    while (nsteps >= 1)
    {
        moveInDirection(map, 'f');
        nsteps--;
    }
    return;
}
