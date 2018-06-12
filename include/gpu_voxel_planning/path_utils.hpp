#ifndef PATH_UTILS_HPP
#define PATH_UTILS_HPP

#include <vector>

typedef std::vector<std::vector<double>> Path;

namespace PathUtils
{
    Path densify(const Path &orig, double max_dist);

    double dist(const std::vector<double> &p1, const std::vector<double> &p2);

    // std::vector<double> interpolate(const std::vector<double> &p0, const std::vector<double> &p1,
    //                                 double interp_dist);

    Path followPartial(const Path& path, double dist);

    void printPath(const Path& path);
};



#endif