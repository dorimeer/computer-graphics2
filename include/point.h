#pragma once

#include <ostream>
#include <string>

struct point
{
    double x;
    double y;
    double z;
};

// printing
std::ostream &operator<<(std::ostream &out, point p);
bool operator!=(const point lhs, const point rhs);
point operator-(point lhs, point rhs);
point operator+(point lhs, point rhs);
point operator*(point lhs, double rhs);
point operator*(point lhs, point rhs);
point operator/(point lhs, double rhs);
point operator/(point lhs, point rhs);
double dot_product(point lhs, point rhs);
point cross_product(point lhs, point rhs);

namespace std
{
string to_string(point p);
point min(point lhs, point rhs);
point max(point lhs, point rhs);
} // namespace std