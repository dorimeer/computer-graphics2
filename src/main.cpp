#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using namespace std::chrono;

const double eps = 1e-7;

struct point
{
    double x;
    double y;
    double z;
};

std::ostream &operator<<(std::ostream &out, point p)
{
    return out << '{' << p.x << ' ' << p.y << ' ' << p.z << '}';
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

struct triangle
{
    std::array<point, 3> vertexes;
    point &operator[](size_t idx) { return vertexes[idx]; }
    point min() { return std::min(vertexes[0], std::min(vertexes[1], vertexes[2])); }
    point max() { return std::max(vertexes[0], std::max(vertexes[1], vertexes[2])); }
};

struct color
{
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};

const bool operator==(const color lhs, const color rhs)
{
    return lhs.red == rhs.red && lhs.green == rhs.green && lhs.blue == rhs.blue;
}

void save_to_file(const std::vector<std::vector<color>> &image, const std::string &file)
{
    using namespace std;
    size_t width = image.size();
    size_t height = image[0].size();

    size_t filesize = 54 + 3 * width * height;
    array<unsigned char, 14> bmp_file_header = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
    array<unsigned char, 40> bmp_info_header = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24};

    bmp_file_header[2] = (unsigned char)(filesize);
    bmp_file_header[3] = (unsigned char)(filesize >> 8);
    bmp_file_header[4] = (unsigned char)(filesize >> 16);
    bmp_file_header[5] = (unsigned char)(filesize >> 24);

    bmp_info_header[4] = (unsigned char)(width);
    bmp_info_header[5] = (unsigned char)(width >> 8);
    bmp_info_header[6] = (unsigned char)(width >> 16);
    bmp_info_header[7] = (unsigned char)(width >> 24);
    bmp_info_header[8] = (unsigned char)(height);
    bmp_info_header[9] = (unsigned char)(height >> 8);
    bmp_info_header[10] = (unsigned char)(height >> 16);
    bmp_info_header[11] = (unsigned char)(height >> 24);

    ofstream fout(file);
    copy(bmp_file_header.begin(), bmp_file_header.end(), ostream_iterator<unsigned char>(fout));
    copy(bmp_info_header.begin(), bmp_info_header.end(), ostream_iterator<unsigned char>(fout));

    for (int j = 0; j < image[0].size(); j++)
    {
        for (int i = 0; i < image.size(); i++)
        {
            if((color)image[i][j] == color{0, 255, 255}) std::cout << i << ' ' << j << '\n';
            fout << image[i][j].blue << image[i][j].green << image[i][j].red;
        }
        // for (size_t i = 0; i < (4 - 3 * image.size() % 4) % 4; i++)
        //     fout << 0;
    }
}

double sqr(double arg)
{
    return arg * arg;
}

point cross_product(point lhs, point rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double area(triangle arg)
{
    auto diamond = cross_product(arg[0] - arg[1], arg[0] - arg[2]);
    return sqrtl(sqr(diamond.x) + sqr(diamond.y) + sqr(diamond.z)) / 2;
}

double dot_product(point lhs, point rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

// first -- distance to intersect
// second -- degree of intersect
std::pair<double, double> intersect(triangle trik, point start, point middle)
{
    point plane_normal = cross_product((trik[0] - trik[1]), (trik[2] - trik[1]));
    point ray_normal = middle - start;

    // does ray and plane intersect?
    double prod1 = dot_product(plane_normal, ray_normal);
    if (std::fabs(prod1) < eps) return {10000, -2};

    // find intersect of ray and plane
    double t = dot_product(plane_normal, trik[0] - start) / prod1;
    point inter = ray_normal * t + start;

    // does triangle contain intersect
    if (std::fabs(area(trik) - area({trik[0], trik[1], inter}) - area({trik[0], trik[2], inter}) -
                  area({trik[2], trik[1], inter})) > eps)
        return {10000, -2};

    return {std::hypot(start.x - inter.x, start.y - inter.y, start.z - inter.z),
            std::fabs(prod1) / (std::hypot(plane_normal.x, plane_normal.y, plane_normal.z) *
                                std::hypot(ray_normal.x, ray_normal.y, ray_normal.z))};
}

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
    // first -- min
    // second -- max
    std::pair<point, point> unite(node *now, triangle trik)
    {
        using std::max;
        using std::min;
        if (!now) return {point{1e10, 1e10, 1e10}, {-1e10, -1e10, -1e10}};
        auto min_trik = trik.min();
        auto max_trik = trik.max();
        return {{min(min_trik.x, now->min.x), min(min_trik.y, now->min.y), min(min_trik.y, now->min.y)},
                {max(max_trik.x, now->max.x), max(max_trik.y, now->max.y), max(max_trik.y, now->max.y)}};
    }

    // first -- area
    // second -- perimeter
    std::pair<double, double> get_unite_param(node *first, node *second, node *rest)
    {
        auto bounding_size = std::max(first->max, second->max) - std::min(first->max, second->max);
        auto rest_size = rest->max - rest->min;
        return {bounding_size.x * bounding_size.y * bounding_size.z + rest_size.x * rest_size.y * rest_size.z,
                bounding_size.x + bounding_size.y + bounding_size.z + rest_size.x + rest_size.y + rest_size.z};
    }

    bool first_more(std::pair<double, double> lhs, std::pair<double, double> rhs)
    {
        if (std::fabs(lhs.first - rhs.first) < eps) return lhs.second > rhs.second;
        return lhs.first > rhs.first;
    }

    std::unique_ptr<node> insert(node *now, triangle new_elem)
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
        bool take_left = 1;
        if (left.x * left.y * left.z < right.x * right.y * right.z) take_left = 0;
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

    void show(node *now, std::string &prefix)
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
    bool intersect(point min_point, point max_point, point start, point middle)
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
        if (min_point.y <= x0 && x0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
        x0 = (max_point.y - start.y) / (middle.y - start.y) * (middle.x - start.x) + start.x;
        z0 = (max_point.y - start.y) / (middle.y - start.y) * (middle.z - start.z) + start.z;
        if (min_point.y <= x0 && x0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
        x0 = (min_point.z - start.z) / (middle.z - start.z) * (middle.x - start.x) + start.x;
        z0 = (min_point.z - start.z) / (middle.z - start.z) * (middle.y - start.y) + start.y;
        if (min_point.y <= y0 && y0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
        x0 = (max_point.z - start.z) / (middle.z - start.z) * (middle.x - start.x) + start.x;
        z0 = (max_point.z - start.z) / (middle.z - start.z) * (middle.y - start.y) + start.y;
        if (min_point.y <= y0 && y0 <= max_point.y && min_point.z <= z0 && z0 <= max_point.z) return 1;
        return 0;
    }
    std::optional<triangle> intersect(node *now, point start, point middle)
    {
        if (!now) return std::nullopt;
        if (!intersect(now->min, now->max, start, middle)) return std::nullopt;
        if(!now->left) return now->value;
        auto left = intersect(now->left.get(), start, middle);
        auto right = intersect(now->right.get(), start, middle);
        if (!left.has_value()) return right;
        if (!right.has_value()) return left;
        if (::intersect(left.value(), start, middle) < ::intersect(right.value(), start, middle)) return left;
        return right;
    }

  public:
    std::pair<double, double> intersect(point start, point middle)
    {
        auto trik = intersect(root.get(), start, middle);
        if (!trik.has_value()) return {100000, -2};
        return ::intersect(trik.value(), start, middle);
    }
    void insert(triangle new_elem)
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
    void show()
    {
        std::string prefix = "";
        show(root.get(), prefix);
        std::cout << "---------------------\n";
    }
};

int main()
{
    tree tr;
    // tr.insert({point{-1, 0, -1}, {-1, 0, 1}, {2, 1, 1}});
    // tr.show();
    // auto inter = tr.intersect({0, 100, 00}, {0, 0, 0});
    // std::cout << inter.first << ' ' << inter.second << '\n';
    // return 0;
    std::ifstream fin("cow.obj");
    std::string line;
    std::vector<point> vertexes;
    std::vector<triangle> triangles;
    char c;
    auto start = high_resolution_clock::now();
    while (fin >> line)
    {
        if (line == "v")
        {
            double x, y, z;
            fin >> x >> y >> z;
            vertexes.push_back({x, y, z});
        }
        if (line == "f")
        {
            size_t ver1, ver2, ver3;
            fin >> ver1 >> c >> c >> ver2 >> ver2 >> c >> c >> ver3 >> ver3 >> line;
            // triangles.push_back({{vertexes[ver1 - 1], vertexes[ver2 - 1], vertexes[ver3 - 1]}});
            tr.insert({{vertexes[ver1 - 1], vertexes[ver2 - 1], vertexes[ver3 - 1]}});
        }
    }
            // tr.show();
            // std::cin.get();
    
    // return 0;
    const int64_t height = 700;
    const int64_t width = 700;
    std::vector<std::vector<color>> image(width, std::vector<color>(height, {0, 0, 0}));
    std::cout << "image was generated\n";
    const size_t proc_num = 11;
    std::vector<std::thread> thrds;
    for (int64_t i = 0; i < width; i++)
        for (int64_t j = 0; j < height; j++)
        {
            std::pair<double, double> res =
                tr.intersect({10, 00, 00}, {0, (2.0 * i - width) / width, (j * 2.0 - height) / height});

            if (res.second == -2) res.second = -1;
            unsigned char color = (1 - std::fabs(res.second)) * 255;
            image[i][j] = {color, color, color};
            if(i == 30)
            {
                image[i][j] = {0, 255, 255};
            }
        }
    /*for (int64_t i = 0; i < width; i++)
        thrds.emplace_back([&triangles, &image, i] {
            for (int64_t j = 0; j < height; j++)
            {
                std::pair<double, double> res = {100000, -2};
                for (auto &trik : triangles)
                {
                    auto inter =
                        intersect(trik, {0, 100, 00}, {(2.0 * i - width) / width, 0, (j * 2.0 - height) / height});
                    res = std::min(res, inter);
                }
                if (res.second == -2) res.second = -1;
                unsigned char color = (1 - std::fabs(res.second)) * 255;
                image[i][j] = {color, color, color};
            }
        });
    for (auto &i : thrds)
        i.join();*/
    auto end = high_resolution_clock::now();
    std::cout << duration_cast<seconds>(end - start).count() << '\n';
    save_to_file(image, "output1.bmp");
}