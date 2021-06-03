#include <point.h>

#include <cmath>
#include <sstream>

#include <constants.h>

bool operator!=(const point lhs, const point rhs)
{
    return std::fabs(lhs.x - rhs.x) > eps || std::fabs(lhs.y - rhs.y) > eps || std::fabs(lhs.z - rhs.z) > eps;
}

std::ostream &operator<<(std::ostream &out, point p)
{
    return out << '{' << p.x << ' ' << p.y << ' ' << p.z << '}';
}

point operator-(point lhs, point rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

point operator+(point lhs, point rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

point operator*(point lhs, double rhs)
{
    return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

point operator*(point lhs, point rhs)
{
    return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

point operator/(point lhs, double rhs)
{
    return {lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
}

point operator/(point lhs, point rhs)
{
    return {lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z};
}

point cross_product(point lhs, point rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double dot_product(point lhs, point rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

namespace std
{
string to_string(point p)
{
    std::stringstream ss;
    ss << p;
    return ss.str();
}

point min(point lhs, point rhs)
{
    return {std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y), std::min(lhs.z, rhs.z)};
}

point max(point lhs, point rhs)
{
    return {std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y), std::max(lhs.z, rhs.z)};
}
} // namespace std