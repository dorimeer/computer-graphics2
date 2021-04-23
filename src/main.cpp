#include <iostream>

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
    std::cout << "Hello world\n";
}