#include <tree.h>

#include <cmath>
#include <iostream>

#include <constants.h>

point get_point_intersect(triangle trik, point start, point middle)
{
    point plane_normal = cross_product((trik[0] - trik[1]), (trik[2] - trik[1]));
    point ray_normal = middle - start;
    double prod1 = dot_product(plane_normal, ray_normal);
    double t = dot_product(plane_normal, trik[0] - start) / prod1;
    point inter = ray_normal * t + start;
    return inter;
}

std::pair<double, double> intersect(triangle trik, point start, point middle)
{
    point plane_normal = cross_product((trik[0] - trik[1]), (trik[2] - trik[1]));
    point ray_normal = middle - start;

    // does ray and plane intersect?
    double prod1 = dot_product(plane_normal, ray_normal);
    if (std::fabs(prod1) < eps) return {10000, -2};

    // // find intersect of ray and plane
    point inter = get_point_intersect(trik, start, middle);

    // does triangle contain intersect
    if (std::fabs(area(trik) - area({trik[0], trik[1], inter}) - area({trik[0], trik[2], inter}) -
                  area({trik[2], trik[1], inter})) > eps)
        return {10000, -2};

    return {std::hypot(start.x - inter.x, start.y - inter.y, start.z - inter.z),
            std::fabs(prod1) / (std::hypot(plane_normal.x, plane_normal.y, plane_normal.z) *
                                std::hypot(ray_normal.x, ray_normal.y, ray_normal.z))};
}

std::pair<point, point> tree::unite(node *now, triangle trik)
{
    using std::max;
    using std::min;
    if (!now) return {point{1e10, 1e10, 1e10}, {-1e10, -1e10, -1e10}};
    auto min_trik = trik.min();
    auto max_trik = trik.max();
    return {{min(min_trik.x, now->min.x), min(min_trik.y, now->min.y), min(min_trik.y, now->min.y)},
            {max(max_trik.x, now->max.x), max(max_trik.y, now->max.y), max(max_trik.y, now->max.y)}};
}

std::pair<double, double> tree::get_unite_param(node *first, node *second, node *rest)
{
    auto bounding_size = std::max(first->max, second->max) - std::min(first->max, second->max);
    auto rest_size = rest->max - rest->min;
    return {bounding_size.x * bounding_size.y * bounding_size.z + rest_size.x * rest_size.y * rest_size.z,
            bounding_size.x + bounding_size.y + bounding_size.z + rest_size.x + rest_size.y + rest_size.z};
}

bool tree::first_more(std::pair<double, double> lhs, std::pair<double, double> rhs)
{
    if (std::fabs(lhs.first - rhs.first) < eps) return lhs.second > rhs.second;
    return lhs.first > rhs.first;
}

std::unique_ptr<tree::node> tree::insert(node *now, triangle new_elem)
{
    // if vertex is leaf
    if (!now->left) return std::make_unique<node>(new_elem);

    // update bounding boxes
    now->min = std::min(now->min, new_elem.min());
    now->max = std::max(now->max, new_elem.max());

    // how child changes if we insert in them
    auto left = unite(now->left.get(), new_elem).second - unite(now->left.get(), new_elem).first;
    auto right = unite(now->right.get(), new_elem).second - unite(now->right.get(), new_elem).first;

    // take best child for insert
    bool take_left = true;
    if (left.x * left.y * left.z < right.x * right.y * right.z) take_left = false;
    if (std::fabs(left.x * left.y * left.z - right.x * right.y * right.z) < eps &&
        right.x + right.y + right.z < left.x + left.y + left.z)
        take_left = 0;
    std::unique_ptr<node> new_child;
    if (take_left)
        new_child = insert(now->left.get(), new_elem);
    else
        new_child = insert(now->right.get(), new_elem);

    // if childs is balanced
    if (!new_child) return nullptr;
    
    if (!now->right)
    {
        now->right = std::move(new_child);
        return nullptr;
    }

    // if we got third child
    // decide which node shpuld be our child and which not
    auto rest_new = get_unite_param(now->left.get(), now->right.get(), new_child.get());
    auto rest_left = get_unite_param(now->right.get(), new_child.get(), now->left.get());
    auto rest_right = get_unite_param(now->left.get(), new_child.get(), now->right.get());
    if (first_more(rest_left, rest_new) && first_more(rest_left, rest_right))
        std::swap(now->left, new_child);
    else if (first_more(rest_right, rest_new) && first_more(rest_right, rest_left))
        std::swap(now->left, new_child);

    now->min = std::min(now->left->min, now->right->min);
    now->max = std::max(now->left->max, now->right->max);

    // return new child for parent
    return std::make_unique<node>(std::move(new_child));
}

void tree::show(node *now, std::string &prefix)
{
    if (!now) return;
    if (!prefix.empty()) prefix.back() = '-';
    std::cout << prefix << now->min << ' ' << now->max << '\n';
    if (!prefix.empty()) prefix.back() = ' ';

    prefix.push_back(' ');
    show(now->left.get(), prefix);
    show(now->right.get(), prefix);
    prefix.pop_back();
}

bool tree::intersect(point min_point, point max_point, point start, point middle)
{
    double y0, x0, z0;
    y0 = (min_point.x - start.x) / (middle.x - start.x) * (middle.y - start.y) + start.y;
    z0 = (min_point.x - start.x) / (middle.x - start.x) * (middle.z - start.z) + start.z;
    if (min_point.y <= y0 && y0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
    y0 = (max_point.x - start.x) / (middle.x - start.x) * (middle.y - start.y) + start.y;
    z0 = (max_point.x - start.x) / (middle.x - start.x) * (middle.z - start.z) + start.z;
    if (min_point.y <= y0 && y0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
    x0 = (min_point.y - start.y) / (middle.y - start.y) * (middle.x - start.x) + start.x;
    z0 = (min_point.y - start.y) / (middle.y - start.y) * (middle.z - start.z) + start.z;
    if (min_point.x <= x0 && x0 <= max_point.x && min_point.z <= z0 && z0 <= max_point.z) return 1;
    x0 = (max_point.y - start.y) / (middle.y - start.y) * (middle.x - start.x) + start.x;
    z0 = (max_point.y - start.y) / (middle.y - start.y) * (middle.z - start.z) + start.z;
    if (min_point.x <= x0 && x0 <= max_point.x && min_point.z <= z0 && z0 <= max_point.z) return 1;
    x0 = (min_point.z - start.z) / (middle.z - start.z) * (middle.x - start.x) + start.x;
    y0 = (min_point.z - start.z) / (middle.z - start.z) * (middle.y - start.y) + start.y;
    if (min_point.x <= x0 && x0 <= max_point.x && min_point.y <= y0 && y0 <= max_point.y) return 1;
    x0 = (max_point.z - start.z) / (middle.z - start.z) * (middle.x - start.x) + start.x;
    y0 = (max_point.z - start.z) / (middle.z - start.z) * (middle.y - start.y) + start.y;
    if (min_point.x <= x0 && x0 <= max_point.x && min_point.y <= y0 && y0 <= max_point.y) return 1;
    return 0;
}

std::optional<triangle> tree::intersect(node *now, point start, point middle)
{
    // std::cout << now << '\n';
    if (!now) return std::nullopt;
    if (!intersect(now->min, now->max, start, middle)) return std::nullopt;
    if (!now->left) return now->value;
    auto left = intersect(now->left.get(), start, middle);
    auto right = intersect(now->right.get(), start, middle);
    if (!left.has_value()) return right;
    if (!right.has_value()) return left;
    if (::intersect(left.value(), start, middle) < ::intersect(right.value(), start, middle)) return left;
    return right;
}

// return -2 if there is no intersect
double tree::intersect(point start, point middle, point light)
{
    auto root_ptr = root.get();
    // insert into root
    auto trik = intersect(root_ptr, start, middle);
    // if there is no intersect triangle return -2
    if (!trik.has_value()) return -2;
    auto res = ::intersect(trik.value(), start, middle);
    if (res.second == -2) return -2;
    // get point intersect
    point inter = get_point_intersect(trik.value(), start, middle);
    // get triangle where light will intersect out object
    auto shade = intersect(root_ptr, light, inter);
    // if it will not intersect(the ray and triangle is parallel) or will intersect not in out point(so we are in shade)
    // that return dark color
    if (!shade.has_value()) return 0;
    if (inter != get_point_intersect(shade.value(), light, inter)) return 0;
    // otherwwise return sin of angle
    return ::intersect(shade.value(), light, inter).second;
}

void tree::insert(triangle new_elem)
{
    // if first elem
    if (!root)
    {
        root = std::make_unique<node>(new_elem);
        return;
    }

    // insert in root
    auto temp = insert(root.get(), new_elem);

    // if no need rebalance
    if (!temp) return;

    // update root
    auto new_root = std::make_unique<node>(std::move(temp));
    new_root->min = std::min(new_root->min, root->min);
    new_root->max = std::max(new_root->max, root->max);
    new_root->right = std::move(root);
    root = std::move(new_root);
}

void tree::show()
{
    std::string prefix = "";
    show(root.get(), prefix);
    std::cout << "---------------------\n";
}