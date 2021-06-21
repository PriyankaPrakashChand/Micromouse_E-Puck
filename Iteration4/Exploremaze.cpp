#include <string>
#include <vector>

#include "Coordinates.h"
#include "Cell.h"
#include "API.h"

using namespace std;

int mazeSize = 16;
void createMap()
{

    vector<vector<Cell>> map(mazeSize);
    for (int i = 0; i < mazeSize; i++)
    {
        map[i].reserve(mazeSize);
    }
}