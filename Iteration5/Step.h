#pragma once
#include "Coordinates.h"
struct Step

{
    Coordinates start, stop;
    Step(Coordinates begin, Coordinates end)
    {
        start = begin;
        stop = end;
    }
};