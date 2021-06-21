#include "IMazeReader.h"
#include <vector>
#include "Cell.h"

using namespace std;

class FloodFillMapReader : public IMazeReader
{
public:
    FloodFillMapReader(vector<vector<Cell>> &map, Coordinates start, Coordinates goal, int width, int height)
    {
        this->map = map;
        this->goal = goal;
        this->width = width;
        this->height = height;
        this->start = start;
    }

    void setWidth(int width)
    {
        this->width = width;
    }

    void setHeight(int height)
    {
        this->height = height;
    }
    Coordinates getGoal()
    {
        return this->goal;
    }
    Coordinates getStart()
    {
        return this->start;
    }
    int getHeight()
    {
        return this->height;
    }
    int getWidth()
    {
        return this->width;
    }
    void setGoal(Coordinates goal)
    {
        this->goal = goal;
    }
    void setStart(Coordinates start)
    {
        this->start = start;
    }
    void isVertex(int y, int x, bool &top, bool &down, bool &left, bool &right, int &isGoal)
    {
        y = this->height - y - 1;
        if ((x == this->goal.x && y == this->goal.y) && (this->start.x == x && y == this->start.y))
            isGoal = 3;
        else if (x == this->goal.x && y == this->goal.y)
            isGoal = 2;
        else if (this->start.x == x && y == this->start.y)
            isGoal = 1;
        else
            isGoal = 0;

        top = !map[x][y].hasNorthWall();
        down = !map[x][y].hasSouthWall();
        left = !map[x][y].hasWestWall();
        right = !map[x][y].hasEastWall();
    }

    void setSize(size_t &row, size_t &col)
    {
        col = this->width;
        row = this->height;
    }

private:
    vector<vector<Cell>> map;
    Coordinates goal;
    Coordinates start;
    int width, height;
};