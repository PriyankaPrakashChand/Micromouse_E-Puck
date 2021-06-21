
#pragma once

struct Coordinates

{
    int x, y;
    // helper functions
    bool operator==(const Coordinates &c)
    {
        return x == c.x && y == c.y;
    }

    bool operator!=(const Coordinates &c)
    {
        return !(this->operator==(c));
    }

    bool operator==(const Coordinates &c) const
    {
        return (x == c.x && y == c.y);
    }
};

bool operator<(const Coordinates &a, const Coordinates &c)
{
    return std::tie(a.x, a.y) < std::tie(c.x, c.y);
}