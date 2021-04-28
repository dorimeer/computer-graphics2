#pragma once

#include <array>

#include <point.h>

struct triangle
{
    std::array<point, 3> vertexes;
    point &operator[](size_t idx);
    point min();
    point max();
};

double area(triangle arg);