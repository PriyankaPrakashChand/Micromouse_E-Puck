/*
 Sample code from https://www.redblobgames.com/pathfinding/a-star/
 Copyright 2014 Red Blob Games <redblobgames@gmail.com>
 
 Feel free to use this code in your own projects, including commercial projects
 License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
*/

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>

struct SimpleGraph
{
    std::unordered_map<char, std::vector<char>> edges;

    std::vector<char> neighbors(char id)
    {
        return edges[id];
    }
};

struct GridLocation
{
    int x, y;
};

namespace std
{
    /* implement hash function so we can put GridLocation into an unordered_set */
    template <>
    struct hash<GridLocation>
    {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation &id) const noexcept
        {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
} // namespace std

struct SquareGrid
{
    static std::array<GridLocation, 4> DIRS;

    int width, height;
    std::unordered_set<GridLocation> walls;

    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const
    {
        return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const
    {
        return walls.find(id) == walls.end();
    }

    std::vector<GridLocation> neighbors(GridLocation id) const
    {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS)
        {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next))
            {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0)
        {
            // see "Ugly paths" section for an explanation:
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

std::array<GridLocation, 4> SquareGrid::DIRS = {
    /* East, West, North, South */
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{0, -1}, GridLocation{0, 1}};

// Helpers for GridLocation

bool operator==(GridLocation a, GridLocation b)
{
    return a.x == b.x && a.y == b.y;
}

bool operator!=(GridLocation a, GridLocation b)
{
    return !(a == b);
}

bool operator<(GridLocation a, GridLocation b)
{
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream &operator<<(std::basic_iostream<char>::basic_ostream &out, const GridLocation &loc)
{
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}
struct GridWithWeights : SquareGrid
{
    std::unordered_set<GridLocation> forests;
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    double cost(GridLocation from_node, GridLocation to_node) const
    {
        return forests.find(to_node) != forests.end() ? 5 : 1;
    }
};

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

template <typename Location>
std::vector<Location> reconstruct_path(
    Location start, Location goal,
    std::unordered_map<Location, Location> came_from)
{
    std::vector<Location> path;
    Location current = goal;
    while (current != start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

// manhattan heuistic
inline double heuristic(GridLocation current, GridLocation goal)
{
    int D = 1;
    return D * std::abs(current.x - goal.x) + std::abs(current.y - goal.y);
}

template <typename Location, typename Graph>
void a_star_search(Graph graph,
                   Location start,
                   Location goal,
                   std::unordered_map<Location, Location> &came_from,
                   std::unordered_map<Location, double> &cost_so_far)
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty())
    {
        Location current = frontier.get();

        if (current == goal)
        {
            break;
        }

        for (Location next : graph.neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}
