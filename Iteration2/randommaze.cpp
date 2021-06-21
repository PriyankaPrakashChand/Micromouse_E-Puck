#include <iostream>
#include <stack>
#include <vector>
#include <random>
#include <string>
#include <fstream>
using namespace std;

// Default values
int m = 4, n = 4;

const double TITLE_SIZE = 0.10;

const double UNIT_SIZE = TITLE_SIZE / 2.0;

// #define out cout

void displayMaze(int M, int N, char **maze)
{

    ofstream out("maze.wbt");

    double floor_M = TITLE_SIZE * m;
    double floor_N = TITLE_SIZE * n;

    out << "#VRML_SIM R2020b utf8 \n"
           " WorldInfo {  \n"
           "  info [  \n"
           "    \"The model of the E-puck robot\"  \n"
           "  ]  \n"
           "  title \"E-puck simulation\"  \n"
           "  coordinateSystem \"NUE\"  \n"
           "}  \n"
           "Viewpoint {  \n"
           "  orientation 1 0 0 4.71238898038469  \n"
           "  position 1.0003051560171443 4.467528391707541 1.0000035012622697  \n"
           "  follow \"e-puck\"  \n"
           "}  \n"
           "TexturedBackground {  \n"
           "}  \n"
           "TexturedBackgroundLight {  \n"
           "}  \n"
           "RectangleArena {  \n"
           "  translation  "
        << floor_N / 2.0  << " 0  " << floor_M / 2.0 << "  \nfloorSize " << floor_N << " " << floor_M << "\n  floorTileSize " << TITLE_SIZE << " " << TITLE_SIZE << 
                                                                                                      "\n floorAppearance Parquetry { "
                                                                                                      "\n    type \"dark strip\" "
                                                                                                      "\n  } "
                                                                                                      "\n} \n DEF JEREMY E-puck \n{\n translation " << TITLE_SIZE/2.0 << " 0 "  << floor_M - (TITLE_SIZE/2.0) << "\n}\n";

    int counter = 0;

    for (int i = 1; i < M - 1; ++i)
    {
        for (int j = 0; j < N - 1; j++)
        {
            if ((maze[i][j] == '#' && maze[i][j + 1] == '#') || (false && 0 < j && maze[i][j - 1] == '#' && maze[i][j] == '#'))
            {
                out << "MazeWall {  \n translation " << (j + 0.5) * UNIT_SIZE << " 0 " << i * UNIT_SIZE << "\nname \"maze wall(" << counter++ << ")\" \n}" << endl;
            }
        }
    }

    for (int i = 0; i < M - 1; ++i)
    {
        for (int j = 1; j < N - 1; j++)
        {
            if ((maze[i][j] == '#' && maze[i + 1][j] == '#') || (false && 0 < i && maze[i - 1][j] == '#' && maze[i][j] == '#'))
            {
                out << "MazeWall {  \n  translation " << j * UNIT_SIZE << " 0 " << (i + 0.5) * UNIT_SIZE << "\n  rotation 0 1 0 -1.570796326 \n name \"maze wall(" << counter++ << " )\" \n  }" << endl;
            }
        }
    }

    for (int i = 0; i < M; i += 1)
    {
        for (int j = 0; j < N; j++)
        {
            cout << maze[i][j] << ' ';
        }
        cout << endl;
    }
}

// A utility function to get the index of cell with indices x, y;
int getIdx(int x, int y, vector<pair<int, pair<int, int>>> cell_list)
{
    for (int i = 0; i < cell_list.size(); i++)
    {
        if (cell_list[i].second.first == x && cell_list[i].second.second == y)
            return cell_list[i].first;
    }
    cout << "getIdx() couldn't find the index!" << endl;
    return -1;
}

void createMaze(int M, int N, char **maze)
{

    vector<pair<int, pair<int, int>>> cell_list;
    vector<bool> visited(m * n, false);
    stack<pair<int, pair<int, int>>> m_stack;
    random_device rdev;
    mt19937 rng(rdev());
    uniform_int_distribution<mt19937::result_type> dist100(1, 100);

    int nVisited = 0;
    int k = 0;

    for (int i = 1; i < M; i += 2)
    {
        for (int j = 1; j < N; j += 2)
        {
            cell_list.push_back(make_pair(k, make_pair(i, j)));
            k++;
        }
    }

    // Generate goal cell

    int top_left = (((m / 2) - 1) * n) + (n / 2) - 1;
    int top_right = top_left + 1;
    int down_left = top_left + n;
    int down_right = down_left + 1;

    // top left
    maze[cell_list[top_left].second.first + 1][cell_list[top_left].second.second + 0] = ' ';
    maze[cell_list[top_left].second.first + 0][cell_list[top_left].second.second + 1] = ' ';
    visited[cell_list[top_left].first] = true;
    nVisited++;

    // top right
    maze[cell_list[top_right].second.first + 1][cell_list[top_right].second.second + 0] = ' ';
    visited[cell_list[top_right].first] = true;
    nVisited++;

    // down left
    maze[cell_list[down_left].second.first + 0][cell_list[down_left].second.second + 1] = ' ';
    visited[cell_list[down_left].first] = true;
    nVisited++;

    // down right
    visited[cell_list[down_right].first] = true;
    nVisited++;

    // Open a single cell to the goal maze

    int randIdx = -1;

    int openFrom = dist100(rng) % 8;

    switch (openFrom)
    {
    case 0: // open top right -> North
        randIdx = top_left - n;
        maze[cell_list[randIdx].second.first + 1][cell_list[randIdx].second.second + 0] = ' ';
        break;
    case 1: // open top right -> North
        randIdx = top_right - n;
        maze[cell_list[randIdx].second.first + 1][cell_list[randIdx].second.second + 0] = ' ';
        break;
    case 2: // open top right -> East
        randIdx = top_right + 1;
        maze[cell_list[randIdx].second.first + 0][cell_list[randIdx].second.second - 1] = ' ';
        break;
    case 3: // open bottom right -> East
        randIdx = down_right + 1;
        maze[cell_list[randIdx].second.first + 0][cell_list[randIdx].second.second - 1] = ' ';
        break;
    case 4: // open bottom right -> South
        randIdx = down_right + n;
        maze[cell_list[randIdx].second.first - 1][cell_list[randIdx].second.second + 0] = ' ';
        break;
    case 5: // open bottom left -> South
        randIdx = down_left + n;
        maze[cell_list[randIdx].second.first - 1][cell_list[randIdx].second.second + 0] = ' ';
        break;
    case 6: // open bottom left -> West
        randIdx = down_left - 1;
        maze[cell_list[randIdx].second.first + 0][cell_list[randIdx].second.second + 1] = ' ';
        break;
    case 7: // open top left -> West
        randIdx = top_left - 1;
        maze[cell_list[randIdx].second.first + 0][cell_list[randIdx].second.second + 1] = ' ';
        break;
    }

    m_stack.push(cell_list[randIdx]);
    visited[randIdx] = true;
    nVisited++;

    // Algo
    while (nVisited < m * n)
    {

        vector<int> neighbours;
        // North
        if (m_stack.top().second.first > 1)
        {
            if (maze[m_stack.top().second.first - 2][m_stack.top().second.second + 0] &&
                !visited[getIdx(m_stack.top().second.first - 2, m_stack.top().second.second + 0, cell_list)])
            {
                neighbours.push_back(0);
            }
        }
        // East
        if (m_stack.top().second.second < N - 2)
        {
            if (maze[m_stack.top().second.first + 0][m_stack.top().second.second + 2] &&
                !visited[getIdx(m_stack.top().second.first + 0, m_stack.top().second.second + 2, cell_list)])
            {
                neighbours.push_back(1);
            }
        }
        // South
        if (m_stack.top().second.first < M - 2)
        {
            if (maze[m_stack.top().second.first + 2][m_stack.top().second.second + 0] &&
                !visited[getIdx(m_stack.top().second.first + 2, m_stack.top().second.second + 0, cell_list)])
            {
                neighbours.push_back(2);
            }
        }
        // West
        if (m_stack.top().second.second > 1)
        {
            if (maze[m_stack.top().second.first + 0][m_stack.top().second.second - 2] &&
                !visited[getIdx(m_stack.top().second.first + 0, m_stack.top().second.second - 2, cell_list)])
            {
                neighbours.push_back(3);
            }
        }
        // Neighbours available?
        if (!neighbours.empty())
        {
            // Choose a random direction
            int next_cell_dir = neighbours[dist100(rng) % neighbours.size()];
            // Create a path between this cell and neighbour
            switch (next_cell_dir)
            {
            case 0: // North
                maze[m_stack.top().second.first - 1][m_stack.top().second.second + 0] = ' ';
                m_stack.push(cell_list[getIdx(m_stack.top().second.first - 2, m_stack.top().second.second + 0, cell_list)]);
                break;
            case 1: // East
                maze[m_stack.top().second.first + 0][m_stack.top().second.second + 1] = ' ';
                m_stack.push(cell_list[getIdx(m_stack.top().second.first + 0, m_stack.top().second.second + 2, cell_list)]);
                break;
            case 2: // South
                maze[m_stack.top().second.first + 1][m_stack.top().second.second + 0] = ' ';
                m_stack.push(cell_list[getIdx(m_stack.top().second.first + 2, m_stack.top().second.second + 0, cell_list)]);
                break;
            case 3: // West
                maze[m_stack.top().second.first + 0][m_stack.top().second.second - 1] = ' ';
                m_stack.push(cell_list[getIdx(m_stack.top().second.first + 0, m_stack.top().second.second - 2, cell_list)]);
                break;
            }

            visited[m_stack.top().first] = true;
            nVisited++;
        }
        else
        {
            m_stack.pop();
        }
    }
}

int main(int argc, char const *argv[])
{
    cout << "Random Maze Generator!" << endl;
    cout << "Enter the order of maze you want (rows (> 4) x cols (> 4)): ";
    cin >> m >> n;
    while (m < 4 || n < 4)
    {
        cout << "Desired dimensions impossible. Re-enter pls." << endl;
        cin >> m >> n;
    }

    int M = 2 * m + 1;
    int N = 2 * n + 1;
    char **maze;
    maze = new char *[M];

    for (int i = 0; i < M; i++)
    {
        maze[i] = new char[N];
    }

    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (!(i & 1) || !(j & 1))
                maze[i][j] = '#';
            else
                maze[i][j] = ' ';
        }
    }

    for (int i = 1; i < M; i += 2)
    {
        for (int j = 1; j < N; j += 2)
        {
            maze[i][j] = ' ';
        }
    }

    createMaze(M, N, maze);
    cout << "Here's the maze you asked for. A maze.wbt is been created. Enjoy! :D" << endl;
    displayMaze(M, N, maze);

    return 0;
}