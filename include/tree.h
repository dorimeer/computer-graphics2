#pragma once

#include <memory>
#include <optional>
#include <tuple>

#include <point.h>
#include <triangle.h>

point get_point_intersect(triangle trik, point start, point middle);

// first -- distance to intersect
// second -- degree of intersect
std::pair<double, double> intersect(triangle trik, point start, point middle);

class tree
{
    struct node
    {
        point min = {1e10, 1e10, 1e10};
        point max = {-1e10, -1e10, -1e10};
        std::unique_ptr<node> right = nullptr;
        std::unique_ptr<node> left = nullptr;
        triangle value; // UB if vertex has childs
        node(triangle elem) : min{elem.min()}, max{elem.max()}, value{elem} {}
        node(std::unique_ptr<node> &&arg) : min{arg->min}, max{arg->max}, left{std::move(arg)} {}
    };
    std::unique_ptr<node> root = nullptr;

    // return cube which contain node and triangle
    // first -- min
    // second -- max
    std::pair<point, point> unite(node *now, triangle trik);

    // return area and perimeter of unite
    // first -- area
    // second -- perimeter
    std::pair<double, double> get_unite_param(node *first, node *second, node *rest);

    bool first_more(std::pair<double, double> lhs, std::pair<double, double> rhs);

    // insert new_elem into node
    std::unique_ptr<node> insert(node *now, triangle new_elem);

    void show(node *now, std::string &prefix);

    // return true if ray {start, middle} intersect cube with bounding boxes {min_point, max_point}
    bool intersect(point min_point, point max_point, point start, point middle);

    // return triangle of intersect
    std::optional<triangle> intersect(node *now, point start, point middle);

  public:
    double intersect(point start, point middle, point light);
    void insert(triangle new_elem);
    void show();
};
