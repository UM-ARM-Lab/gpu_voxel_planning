#include "gpu_voxel_planning/path_utils.hpp"
#include <cassert>
#include <stddef.h>
#include <math.h>
#include <arc_utilities/eigen_helpers.hpp>
// #include <arc_utilities/shortcut_smoothing.hpp>
#include <arc_utilities/path_utils.hpp>

using namespace PathUtils;


Path PathUtils::densify(const Path &path, double max_dist)
{
    const auto interp_fn = [&](const std::vector<double> &v1, const std::vector<double> &v2, const double ratio)
    {
        return EigenHelpers::Interpolate(v1, v2, ratio);
    };

    const auto dist_fn = [](const std::vector<double> &p1, const std::vector<double> &p2)
    {
        return PathUtils::dist(p1, p2);
    };
    
    Path p = path_utils::ResamplePath(path, max_dist, dist_fn, interp_fn);
    // Path p = shortcut_smoothing::ResamplePath(path, max_dist, dist, EigenHelpers::Interpolate);
    return p;
}


double PathUtils::dist(const std::vector<double> &p1, const std::vector<double> &p2)
{
    assert(p1.size() == p2.size());
    double d = 0;
    for(size_t i=0; i<p1.size(); i++)
    {
        d+= (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }
    return sqrt(d);
}

double PathUtils::length(const Path& path)
{
    if(path.size() <= 1)
    {
        return 0;
    }

    double path_length = 0;
    for(int i=0; i+1 < path.size(); i++)
    {
        path_length += dist(path[i], path[i+1]);
    }
    return path_length;
}


size_t PathUtils::closestIndex(const Path &path, const std::vector<double> &point)
{
    assert(path.size() > 0);
    double d = dist(path[0], point);
    size_t ind = 0;

    for(size_t i=1; i<path.size(); i++)
    {
        if(dist(path[i], point) < d)
        {
            d = dist(path[i], point);
            ind = i;
        }
    }
    return ind;
}

Path PathUtils::followPartial(const Path& path, double max_dist)
{
    Path new_path;
    if(path.size() == 0)
    {
        return path;
    }
    new_path.push_back(path[0]);
    
    double dist_so_far = 0;
    for(size_t i=1; i < path.size(); i++)
    {
        double inc_dist = PathUtils::dist(path[i-1], path[i]);
        double dist_left = max_dist - dist_so_far;
        if(inc_dist > dist_left)
        {
            new_path.push_back(EigenHelpers::Interpolate(path[i-1], path[i], dist_left/inc_dist));
            break;
        }
        new_path.push_back(path[i]);
        dist_so_far += inc_dist;
    }
    return new_path;
}

void PathUtils::printPath(const Path& path)
{
    std::cout << "Path: \n";
    for(auto p: path)
    {
        for(auto v: p)
        {
            std::cout << v << ", ";
        }
        std::cout << "\n";
    }
}

// Path densify(const Path &orig, double max_dist)
// {
//     if(orig.size() == 0)
//     {
//         return orig;
//     }
    
//     double max_dist_sq = max_dist * max_dist;
//     Path new_path;
    

//     for(int i=0; i<(int)orig.size() - 1; i++)
//     {
//         double d = dist(orig[i], orig[i+1]);
//         int num_points_needed = (int)ceil(d/max_dist);
//         double step_size = d/num_points_needed;
//         assert(std_size <= max_dist);
//         for(int j=0; j<num_points_needed; j++)
//         {
//             new_path.push_back(interpolate(orig[i], orig[i+1], j*step_size));
//         }
//     }

//     new_path.push_back(orig.back());
//     return new_path;
// }




// std::vector<double> interpolate(const std::vector<double> &p0, const std::vector<double> &p1,
//                                 double interp_dist)
// {
//     std::vector<double> p_new;
//     double d = dist(p0, p1);
//     double scale = interp_dist/d;

//     for(size_t i=0; i<p0.size(); i++)
//     {
//         p_new.push_back(p0[i] + scale*(p1[i] - p0[i]));
//     }
//     return p_new;
// }


