#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
// #include <thread>
#include <vector>

#include <file_representation.h>
#include <point.h>
#include <tree.h>
// #include <triangle.h>

using namespace std::chrono;

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "wrong number of arguments\nCorrect format is:\n./ray_tracer --source=<path to your *.obj> "
                     "--output=<path to output image>\n";
        return -1;
    }

    // get source and output file
    std::string source;
    std::string output;
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

    // read file and insert in tree

    tree tr;
    std::ifstream fin(source);
    std::string line;
    std::vector<point> vertexes;
    std::vector<point> vertexes_normals;
    while (fin >> line)
    {
        // read every vertex
        if (line == "v")
        {
            double x, y, z;
            fin >> x >> y >> z;
            vertexes.push_back({x, y, z});
        }
        if (line == "vn")
        {
            double x, y, z;
            fin >> x >> y >> z;
            vertexes_normals.push_back({x, y, z});
        }
        // read every triangle
        if (line == "f")
        {
            size_t ver1, ver2, ver3;
            size_t vn1, vn2, vn3;
            fin >> ver1;
            fin.ignore(2, '\\');
            fin >> vn1;
            fin.ignore(10, ' ');
            fin >> ver2;
            fin.ignore(2, '\\');
            fin >> vn2;
            fin.ignore(10, ' ');
            fin >> ver3;
            fin.ignore(2, '\\');
            fin >> vn3;
            fin.ignore(10, '\n');
            triangle triangl = {{vertexes[ver1 - 1],
                           vertexes[ver2 - 1],
                           vertexes[ver3 - 1]},
                           {vertexes_normals[vn1-1],
                            vertexes_normals[vn2-1],
                            vertexes_normals[vn3-1]}};
            tr.insert(triangl);
        }
    }
    auto read_end = high_resolution_clock::now();
    std::cout << "read and insert in tree took " << duration_cast<milliseconds>(read_end - start).count() << "ms\n";

    const int64_t height = 500;
    const int64_t width = 500;
    std::vector<std::vector<color>> image(width, std::vector<color>(height, {0, 0, 0}));

    const point light = {5, 5, 5};
    const point camera = {10, 10, 0};

    double h_pov = 0.6;
    double v_pov = 0.6;

    // calculate every pixel value
    for (int64_t i = 0; i < width; i++)
        for (int64_t j = 0; j < height; j++)
        {
            // here we think that out object is located in coordinate {0, 0, 0}
            // and our camera is looking into square {-1, 0, -1}...{1, 0, 1}

            point plane_point = {(2.0 * i - width) / width * h_pov, 0, (j * 2.0 - height) / height * v_pov};

            double res = tr.intersect(camera, plane_point, light);

            // if -2 than there is no intersection
            // otherwise according to formula
            if (res == -2) continue;
            unsigned char color = (std::fabs(res) + 0.5) / 1.5 * 255;
            image[i][j] = {color, color, color};
        }

    auto end = high_resolution_clock::now();
    std::cout << "ray tracing itself took " << duration_cast<milliseconds>(end - read_end).count() << "ms\n";

    save_to_file(image, output);

    auto total_end = high_resolution_clock::now();
    std::cout << "write to file took " << duration_cast<milliseconds>(total_end - end).count() << "ms\n";
    std::cout << "the whole program took " << duration_cast<milliseconds>(total_end - start).count() << "ms\n";
}