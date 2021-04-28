#include <triangle.h>

#include <cmath>

point &triangle::operator[](size_t idx)
{
    return vertexes[idx];
}
point triangle::min()
{
    return std::min(vertexes[0], std::min(vertexes[1], vertexes[2]));
}
point triangle::max()
{
    return std::max(vertexes[0], std::max(vertexes[1], vertexes[2]));
}

double area(triangle arg)
{
    auto diamond = cross_product(arg[0] - arg[1], arg[0] - arg[2]);
    return std::hypot(diamond.x, diamond.y, diamond.z) / 2;
}