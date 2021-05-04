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

    // read file and insert in tree

    tree tr;
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
            tr.insert({{vertexes[ver1 - 1], vertexes[ver2 - 1], vertexes[ver3 - 1]}});
        }
    }
    auto read_end = high_resolution_clock::now();
    std::cout << "read and insert in tree took " << duration_cast<milliseconds>(read_end - start).count() << "ms\n";

    const int64_t height = 721;
    const int64_t width = 721;
    std::vector<std::vector<color>> image(width, std::vector<color>(height, {0, 0, 0}));

    const point light = {5, 5, 5};
    const point camera = {10, 10, 0};

    // calculate every pixel value
    for (int64_t i = 0; i < width; i++)
        for (int64_t j = 0; j < height; j++)
        {
            // here we think that out object is located in coordinate {0, 0, 0}
            // and our camera is looking into square {-1, 0, -1}...{1, 0, 1}
            double res = tr.intersect(camera, {(2.0 * i - width) / width, 0, (j * 2.0 - height) / height}, light);

            // if -2 than there is no intersection
            // otherwise according to formula
            if (res == -2) continue;
            unsigned char color = (std::fabs(res) + 0.5) / 1.5 * 255;
            image[i][j] = {0, 0, color};
        }

    auto end = high_resolution_clock::now();
    std::cout << "ray tracing itself took " << duration_cast<milliseconds>(end - read_end).count() << "ms\n";

    save_to_file(image, output);

    auto total_end = high_resolution_clock::now();
    std::cout << "write to file took " << duration_cast<milliseconds>(total_end - end).count() << "ms\n";
    std::cout << "the whole program took " << duration_cast<milliseconds>(total_end - start).count() << "ms\n";
}