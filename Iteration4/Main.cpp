#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <stack>

#include "API.h"
#include "Cell.h"

using namespace std;
int numCellsExplored = 0, numNewCellsExplored = 0, numUpdates = 0, numNewUpdates = 0;
int numTurnsTaken = 0, numNewTurnsTaken = 0;
int mazeSize = 16; // 16X16
int centerX = mazeSize / 2;
int centerY = mazeSize / 2;
int startX = 0;
int startY = 0;

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

//---------------------function signatures--------------------
void log(const std::string &text);
void logInline(const std::string &text);
const std::string currentDateTime();
bool isGoal(int x, int y);
bool isSafe(int X, int Y);
void updateposition(char currentMove);
void exploreCell(vector<vector<Cell>> &map, int x, int y);
void initializeMaze(vector<vector<Cell>> &map);
void findGoal(vector<vector<Cell>> &map);
Cell &getFrontCell(vector<vector<Cell>> &map);
Cell &getLeftCell(vector<vector<Cell>> &map);
Cell &getRightCell(vector<vector<Cell>> &map);
void getMinDistanceDirection(vector<vector<Cell>> &map, int &minDistance, char &minDirection);
void floodOpenNeighbours(vector<vector<Cell>> &map);
void moveInDirection(vector<vector<Cell>> &map, char direction);
int countDigit(int n);
void print_maze(vector<vector<Cell>> &map);
//---------------------main()--------------
int main(int argc, char *argv[])
{
    vector<vector<Cell>> map(mazeSize, vector<Cell>(mazeSize));

    log("Start Time = " + currentDateTime());

    initializeMaze(map);
    // print_maze(map);

    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    findGoal(map);

    log("End Time = " + currentDateTime());
    log("Number of turns taken: " + std::to_string(numTurnsTaken));
    log("Number of cells explored: " + std::to_string(numCellsExplored));
    log("Number of Flood Fill Cost Updates: " + std::to_string(numUpdates));

    log("Goal Found..! now moving back t start");
    log("Start Time = " + currentDateTime());
    searchMode = FIND_START;
    findGoal(map);
    log("End Time = " + currentDateTime());
    log("Number of  Extra turns taken: " + std::to_string(numNewTurnsTaken));
    log("Number of Extra cells explored: " + std::to_string(numNewCellsExplored));
    log("Number of Extra Flood Fill Cost Updates: " + std::to_string(numNewUpdates));
}
//---------------------- function defintions-------------------------------
bool isGoal(int x, int y)
{
    if (searchMode == FIND_CENTRE)
        return ((x == 7 || x == 8) && (y == 7 || y == 8));
    else
        return ((x == 0) && (y == 0));
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

void log(const std::string &text)
{
    std::cerr << text << std::endl;
}

void logInline(const std::string &text)
{
    std::cerr << text;
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

void exploreCell(vector<vector<Cell>> &map, int x, int y)
{
    // map[x][y].setVisited(); // first visit is straight, then right, then left, then back
    if (map[x][y].getHasBeenExplored())
    {
        return;
    }

    map[x][y].sethasBeenExplored(true);
    if (searchMode == FIND_CENTRE)
    {
        numCellsExplored++;
    }
    else
    {
        numNewCellsExplored++;
    }
    switch (orientation)
    {
    case 'N':

        map[x][y].setNorthWall(API::wallFront());

        map[x][y].setEastWall(API::wallRight());
        map[x][y].setWestWall(API::wallLeft());

        break;
    case 'S':

        map[x][y].setSouthWall(API::wallFront());
        map[x][y].setEastWall(API::wallLeft());
        map[x][y].setWestWall(API::wallRight());
        break;
    case 'W':

        map[x][y].setWestWall(API::wallFront());
        map[x][y].setSouthWall(API::wallLeft());
        map[x][y].setNorthWall(API::wallRight());
        break;
    case 'E':

        map[x][y].setEastWall(API::wallFront());
        map[x][y].setNorthWall(API::wallLeft());
        map[x][y].setSouthWall(API::wallRight());
        break;
    default:
        int a = 0;
    }
}

void initializeMaze(vector<vector<Cell>> &map)
{

    for (int i = 0; i < mazeSize; i++)
    {
        map[i].reserve(mazeSize);
    }
    for (int i = 0; i < mazeSize; i++)
    {
        for (int j = 0; j < mazeSize; j++)
        {
            map[i][j].setcellAddress(Coordinates{i, j});
            int minX = std::min(std::abs(i - centerX), std::abs(i - (centerX - 1)));

            int minY = std::min(std::abs(j - centerY), std::abs(j - (centerY - 1)));
            int floodFillCost = minX + minY;
            map[i][j].setFloodFillCost(floodFillCost);
            map[i][j].setReverseFloodFillCost(i + j);
        }
    }

    map[0][0].setSouthWall(true); // leads to errors
}

void findGoal(vector<vector<Cell>> &map) // takes coordinates of current cell
{
    bool destinationFound = false;
    int minDistance;
    char minDirection;
    // alias for conveneience
    while (!destinationFound)
    {

        exploreCell(map, x, y); // explore current cell
        minDistance = mazeSize * 2;
        // if goal fount-> exit while loop
        if (isGoal(x, y))
            destinationFound = true;
        if (!destinationFound)
        {
            getMinDistanceDirection(map, minDistance, minDirection);
            // check if reflooding is required

            if (((searchMode == FIND_CENTRE) && map[x][y].getFloodFillCost() != 1 + minDistance) || ((searchMode == FIND_START) && map[x][y].getReverseFloodFillCost() != 1 + minDistance))

            {
                // reflood
                floodOpenNeighbours(map);

                // after reflooding get new min distance neighbours
                getMinDistanceDirection(map, minDistance, minDirection); //get neighbour with lowest distance
            }

            moveInDirection(map, minDirection);
        }
    }
}

// f-move front ,b- move back,r-move right, l-moveleft
void moveInDirection(vector<vector<Cell>> &map, char direction)
{
    // move to the  neighbouring cell with the lowest distance cost
    int prevX = x, prevY = y;
    if (direction == 'f')
    {
        API::moveForward();
        updateposition('f');
    }
    else if (direction == 'l')
    {
        API::turnLeft();
        updateposition('l');
        API::moveForward();
        updateposition('f');
        if (searchMode == FIND_CENTRE)
        {
            numTurnsTaken++;
        }
        else
        {
            numNewTurnsTaken++;
        }
    }

    else if (direction == 'r')
    {

        API::turnRight();
        updateposition('r');
        API::moveForward();
        updateposition('f');

        if (searchMode == FIND_CENTRE)
        {
            numTurnsTaken++;
        }
        else
        {
            numNewTurnsTaken++;
        }
    }
    else if (direction == 'b')
    {
        API::turnRight();
        updateposition('r');
        API::turnRight();
        updateposition('r');
        API::moveForward();
        updateposition('f');
        if (searchMode == FIND_CENTRE)
        {
            numTurnsTaken++;
            numTurnsTaken++;
        }
        else
        {
            numNewTurnsTaken++;
            numNewTurnsTaken++;
        }
    }

    map[x][y]
        .setPrevVisitedCell(&map[prevX][prevY]);

    if (searchMode == FIND_CENTRE)
    {
        API::setColor(x, y, 'Y');
    }
    else
    {
        API::setColor(x, y, 'B');
    }
}
// doesnt check if coordinates are within boundaries
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

//get min distance relative to current position and relative position f = front, b = behind, l=left,r=right
void getMinDistanceDirection(vector<vector<Cell>> &map, int &minDistance, char &minDirection)
{
    switch (searchMode)
    {
    case FIND_CENTRE:
        if (!API::wallFront())
        {
            Cell front = getFrontCell(map);

            if (minDistance > front.getFloodFillCost())
            {
                minDistance = front.getFloodFillCost();
                minDirection = 'f';
            }
        }
        if (!API::wallLeft())
        {
            Cell left = getLeftCell(map);
            if (minDistance > left.getFloodFillCost())
            {
                minDistance = left.getFloodFillCost();
                minDirection = 'l';
            }
        }
        if (!API::wallRight())
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
        break;
    case FIND_START:
        if (!API::wallFront())
        {
            Cell front = getFrontCell(map);

            if (minDistance > front.getReverseFloodFillCost())
            {
                minDistance = front.getReverseFloodFillCost();
                minDirection = 'f';
            }
        }
        if (!API::wallLeft())
        {
            Cell left = getLeftCell(map);
            if (minDistance > left.getReverseFloodFillCost())
            {
                minDistance = left.getReverseFloodFillCost();
                minDirection = 'l';
            }
        }
        if (!API::wallRight())
        {
            Cell right = getRightCell(map);
            if (minDistance > right.getReverseFloodFillCost())
            {
                minDistance = right.getReverseFloodFillCost();
                minDirection = 'r';
            }
        }
        if (map[x][y].getPrevVisitedCell() != nullptr)
        {
            Cell *back = map[x][y].getPrevVisitedCell();
            if (minDistance > back->getReverseFloodFillCost())
            {
                minDistance = back->getReverseFloodFillCost();
                minDirection = 'b';
            }
        }

        break;
    default:
        int a = 0; // do nothing
    }
}

void floodOpenNeighbours(vector<vector<Cell>> &map)
{

    std::stack<Coordinates> floodStack;
    int minDistance = mazeSize * 2;
    char minDirection;
    int cellX, cellY;
    floodStack.push(Coordinates({x, y}));
    while (!floodStack.empty())
    {
        cellX = floodStack.top().x;
        cellY = floodStack.top().y;

        floodStack.pop();
        if (isGoal(cellX, cellY))
            continue;

        Cell cell = map[cellX][cellY];

        if (cell.getHasBeenExplored())
        {
            int D1, D2, D3, D4;
            minDistance = mazeSize * 2;
            if (searchMode == FIND_CENTRE)
            {
                D1 = (!map[cellX][cellY].hasNorthWall()) ? map[cellX][cellY + 1].getFloodFillCost() : mazeSize * 2;
                D2 = (!map[cellX][cellY].hasSouthWall()) ? map[cellX][cellY - 1].getFloodFillCost() : mazeSize * 2;
                D3 = (!map[cellX][cellY].hasWestWall()) ? map[cellX - 1][cellY].getFloodFillCost() : mazeSize * 2;
                D4 = (!map[cellX][cellY].hasEastWall()) ? map[cellX + 1][cellY].getFloodFillCost() : mazeSize * 2;
            }
            else
            {
                D1 = (!map[cellX][cellY].hasNorthWall()) ? map[cellX][cellY + 1].getReverseFloodFillCost() : mazeSize * 2;
                D2 = (!map[cellX][cellY].hasSouthWall()) ? map[cellX][cellY - 1].getReverseFloodFillCost() : mazeSize * 2;
                D3 = (!map[cellX][cellY].hasWestWall()) ? map[cellX - 1][cellY].getReverseFloodFillCost() : mazeSize * 2;
                D4 = (!map[cellX][cellY].hasEastWall()) ? map[cellX + 1][cellY].getReverseFloodFillCost() : mazeSize * 2;
            }

            minDistance = std::min(D1, D2);
            minDistance = std::min(minDistance, D3);
            minDistance = std::min(minDistance, D4);
            //----------add to stack
            if (((searchMode == FIND_CENTRE) && map[cellX][cellY].getFloodFillCost() != 1 + minDistance) || ((searchMode == FIND_START) && map[cellX][cellY].getReverseFloodFillCost() != 1 + minDistance))
            {
                if (searchMode == FIND_CENTRE)
                {
                    map[cellX][cellY].setFloodFillCost(1 + minDistance);
                    numUpdates++;
                }
                else
                {
                    map[cellX][cellY].setReverseFloodFillCost(1 + minDistance);
                    numNewUpdates++;
                }

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
            int d1, d2, d3, d4;
            int minD = mazeSize * 2;
            // a cell that has not been explored has no walls so all neighbours are accessible
            if (searchMode == FIND_CENTRE)
            {
                d1 = isSafe(cellX + 1, cellY) ? map[cellX + 1][cellY].getFloodFillCost() : mazeSize * 2;
                d2 = isSafe(cellX - 1, cellY) ? map[cellX - 1][cellY].getFloodFillCost() : mazeSize * 2;
                d3 = isSafe(cellX, cellY + 1) ? map[cellX][cellY + 1].getFloodFillCost() : mazeSize * 2;
                d4 = isSafe(cellX, cellY - 1) ? map[cellX][cellY - 1].getFloodFillCost() : mazeSize * 2;
            }
            else
            {
                d1 = isSafe(cellX + 1, cellY) ? map[cellX + 1][cellY].getReverseFloodFillCost() : mazeSize * 2;
                d2 = isSafe(cellX - 1, cellY) ? map[cellX - 1][cellY].getReverseFloodFillCost() : mazeSize * 2;
                d3 = isSafe(cellX, cellY + 1) ? map[cellX][cellY + 1].getReverseFloodFillCost() : mazeSize * 2;
                d4 = isSafe(cellX, cellY - 1) ? map[cellX][cellY - 1].getReverseFloodFillCost() : mazeSize * 2;
            }
            minD = std::min(d1, d2);
            minD = std::min(minD, d3);
            minD = std::min(minD, d4);
            if (((searchMode == FIND_CENTRE) && map[cellX][cellY].getFloodFillCost() != 1 + minD) || ((searchMode == FIND_START) && map[cellX][cellY].getReverseFloodFillCost() != 1 + minD))
            {
                if (searchMode == FIND_CENTRE)
                {
                    map[cellX][cellY].setFloodFillCost(1 + minD);
                    numUpdates++;
                }
                else
                {
                    map[cellX][cellY].setReverseFloodFillCost(1 + minD);
                    numNewUpdates++;
                }

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
bool isSafe(int X, int Y)
{
    if (X < 0 || X > mazeSize - 1)
        return false;
    if (Y < 0 || Y > mazeSize - 1)
        return false;
    return true;
}

int countDigit(int n)
{
    int count = 0;
    while (n != 0)
    {
        n = n / 10;
        ++count;
    }
    return count;
}
void print_maze(vector<vector<Cell>> &map)
{
    for (int i = 0; i < mazeSize; i++)
    {
        for (int j = 0; j < mazeSize; j++)
        {
            int cost = map[i][j].getFloodFillCost();
            // int numDigits = countDigit(cost);
            // if (numDigits == 1)
            // {
            //     log("00");
            // }

            // else if (numDigits == 2)
            // {
            //     log("0");
            // }
            log(std::to_string(cost) + " ");
        }
        log("");
    }
}