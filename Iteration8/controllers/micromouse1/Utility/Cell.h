#ifndef __CELL__
#define __CELL__
#include <string>
#include "Coordinates.h"

// using namespace std;

class Cell
{
public:
    Cell()
    {
        // doesnt know its adress initially
        cellAddress.x = 0;
        cellAddress.y = 0;
        // no walls initially
        northWall = false;
        southWall = false;
        westWall = false;
        eastWall = false;
        // have walls been detected
        hasBeenExplored = false;
        // no.of times visited
        visited = 0;
        floodFillCost = -1;
        reverseFloodFillCost = -1;
        prevVisitedCell = nullptr;
    }

    // setters
    void setcellAddress(const Coordinates &coordinates)
    {
        cellAddress = coordinates;
    }
    void setNorthWall(bool wall)
    {
        northWall = wall;
    }
    void setSouthWall(bool wall)
    {
        southWall = wall;
    }
    void setWestWall(bool wall)
    {
        westWall = wall;
    }
    void setEastWall(bool wall)
    {
        eastWall = wall;
    }
    void sethasBeenExplored(bool status)
    {
        hasBeenExplored = status;
    }
    void setFloodFillCost(int cost)
    {
        floodFillCost = cost;
    }
    void setVisited()
    {
        ++visited;
    }
    void setPrevVisitedCell(Cell *prev)
    {
        prevVisitedCell = prev;
    }
    void setReverseFloodFillCost(int cost)
    {
        reverseFloodFillCost = cost;
    }
    // getters
    Coordinates
    getCellAddress()
    {
        return cellAddress;
    }

    bool hasNorthWall()
    {
        return northWall;
    }
    bool hasSouthWall()
    {
        return southWall;
    }
    bool hasWestWall()
    {
        return westWall;
    }
    bool hasEastWall()
    {
        return eastWall;
    }
    int getFloodFillCost()
    {
        return floodFillCost;
    }
    bool getHasBeenExplored()
    {
        return hasBeenExplored;
    }
    int Visited()
    {
        return visited;
    }

    Cell *getPrevVisitedCell()
    {
        return prevVisitedCell;
    }

    int getReverseFloodFillCost()
    {
        return reverseFloodFillCost;
    }
    bool getWallStatus(char c)
    {
        if (c == 'N')
        {
            return northWall;
        }
        else if (c == 'S')
        {
            return southWall;
        }
        else if (c == 'E')
        {
            return eastWall;
        }
        else if (c == 'W')
        {
            return westWall;
        }
        else
            std::cerr << "Error wrong face entered";
        return true;
    }

private:
    Coordinates cellAddress;
    bool northWall, southWall, westWall, eastWall, hasBeenExplored;
    int visited;
    int floodFillCost, reverseFloodFillCost;

    Cell *prevVisitedCell;
};

#endif