#ifndef OPTION_PARSE_H_
#define OPTION_PARSE_H_

//
// From stackoverflow:
// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
//
#include <algorithm>

char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

std::tuple<bool, std::string> get_filename(int argc, char *argv[])
{
    if (cmdOptionExists(argv, argv + argc, "-filename"))
        return std::make_tuple(true, std::string(getCmdOption(argv, argv + argc, "-filename")));
    else
        return std::make_tuple(false, "none");
}

std::vector<double> get_random_vector(const double &max, const double &min, const int &numDim)
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dis(min, max);

    std::vector<double> vec;

    for (int i = 0; i < numDim; i++)
    {
        vec.push_back(dis(gen));
    }
    return vec;
}

#endif // OPTION_PARSE_H_
