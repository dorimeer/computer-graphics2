#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono;

const double eps = 1e-7;

struct point
{
    double x;
    double y;
    double z;
};

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
};

struct color
{
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};

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
    std::for_each(image.rbegin(), image.rend(), [&fout](const auto &line) {
        for (auto &i : line)
            fout << i.blue << i.green << i.red;
        for (size_t i = 0; i < 4 - 3 * line.size() % 4; i++)
            fout << 0;
    });
}

double area(triangle arg)
{
    point a = arg.vertexes[0];
    point b = arg.vertexes[1];
    point c = arg.vertexes[2];

    double x = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    double y = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y) + (a.z - c.z) * (a.z - c.z));
    double z = sqrt((c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y) + (c.z - b.z) * (c.z - b.z));
    double p = (x + y + z) / 2;
    return sqrt(p * (p - x) * (p - y) * (p - z));
}

point cross_product(point lhs, point rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double dot_product(point lhs, point rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double sqr(double arg)
{
    return arg*arg;
}

double intersect(triangle trik, point start, point middle)
{
    point plane_normal = cross_product((trik[0] - trik[1]), (trik[2] - trik[1]));
    point ray_normal = middle - start;

    // does ray and plane intersect?
    double prod1 = dot_product(plane_normal, ray_normal);
    if (abs(prod1) < eps) return -2;

    // find intersect of ray and plane
    double t = dot_product(plane_normal, trik[0] - start) / prod1;
    point inter = ray_normal * t + start;

    std::cout << "intersect: (" << inter.x << ' ' << inter.y << ' ' << inter.z << ")\n";

    // does triangle contain intersect
    std::cout << area(trik) << ' ' << area({trik[0], trik[1], inter})  << ' ' << area({trik[0], trik[2], inter})  << ' ' <<  area({trik[2], trik[1], inter}) << '\n';
    if (abs(area(trik) - area({trik[0], trik[1], inter}) - area({trik[0], trik[2], inter}) - area({trik[2], trik[1], inter})) > eps) return -2;
    
    return sin(abs(prod1)/(sqrt(sqr(plane_normal.x) + sqr(plane_normal.y) + sqr(plane_normal.z))*sqrt(sqr(ray_normal.x) + sqr(ray_normal.y) + sqr(ray_normal.z))));
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "wrong number of arguments\nCorrect format is:\n./ray_tracer --source=<path to your *.obj> "
                     "--output=<path to output image>\n";
        return -1;
    }

    // get source and output file
    std::string source = "";
    std::string output = "";
    for (int i = 1; i < argc; i++)
    {
        std::string_view arg(argv[i]);
        if (arg.starts_with("--source")) source = arg.substr(9);
        if (arg.starts_with("--output")) output = arg.substr(9);
    }
    if (source.empty() || output.empty())
    {
        std::cerr << "wrong arguments\nSource or output file is missing";
        return -1;
    }

    auto start = high_resolution_clock::now();
    // tree tr;
    std::ifstream fin(source);
    std::string line;
    std::vector<point> vertexes;
    char c;
    while (fin >> line)
    {
        // read every vertex
        if (line == "v")
        {
            double x, y, z;
            fin >> x >> y >> z;
            vertexes.push_back({x, y, z});
        }
        // read every triangle
        if (line == "f")
        {
            size_t ver1, ver2, ver3;
            fin >> ver1 >> c >> c >> ver2 >> ver2 >> c >> c >> ver3 >> ver3 >> line;
            // tr.insert({{vertexes[ver1 - 1], vertexes[ver2 - 1], vertexes[ver3 - 1]}});
        }
    }
    auto read_end = high_resolution_clock::now();
    std::cout << "read and insert in tree took " << duration_cast<milliseconds>(read_end - start).count() << "ms\n";
    std::ifstream fin("cow.obj");
    std::string line;
    std::vector<point> vertexes;
    std::vector<triangle> triangles;
    char c;
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
            triangles.push_back({{vertexes[ver1], vertexes[ver2], vertexes[ver3]}});
        }
    }
}