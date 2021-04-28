#pragma once

#include <string>
#include <vector>

struct color
{
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};

const bool operator==(const color lhs, const color rhs);

// takes 2d matrix and file to save
void save_to_file(const std::vector<std::vector<color>> &image, const std::string &file);