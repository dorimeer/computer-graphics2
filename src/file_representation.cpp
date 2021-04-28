#include <file_representation.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>

const bool operator==(const color lhs, const color rhs)
{
    return lhs.red == rhs.red && lhs.green == rhs.green && lhs.blue == rhs.blue;
}

void save_to_file(const std::vector<std::vector<color>> &image, const std::string &file)
{
    using namespace std;
    size_t width = image.size();
    size_t height = image[0].size();
    // store header itself
    size_t filesize = 54 + 3 * width * height;
    std::array<unsigned char, 14> bmp_file_header = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
    std::array<unsigned char, 40> bmp_info_header = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24};

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
    std::copy(bmp_file_header.begin(), bmp_file_header.end(), ostream_iterator<unsigned char>(fout));
    std::copy(bmp_info_header.begin(), bmp_info_header.end(), ostream_iterator<unsigned char>(fout));

    // stores pixels
    for (int j = 0; j < image[0].size(); j++)
    {
        for (int i = 0; i < image.size(); i++)
        {
            fout << image[i][j].blue << image[i][j].green << image[i][j].red;
        }
        for (size_t i = 0; i < (4 - 3 * image.size() % 4) % 4; i++)
            fout << 0;
    }
}