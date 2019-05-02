#ifndef GVP_PATH_UTILS_ADDONS_HPP
#define GVP_PATH_UTILS_ADDONS_HPP

#include "path_utils.hpp"
#include "robot_model.hpp"


namespace GVP
{
    typedef std::vector<VictorRightArmConfig> Path;

    inline PathUtils::Path toPathUtilsPath(const Path &p)
    {
        PathUtils::Path pu;
        for(const auto& config: p)
        {
            pu.push_back(config.asVector());
        }
        return pu;
    }

    inline Path toPath(const PathUtils::Path &pu)
    {
        Path p;
        for(const auto& point: pu)
        {
            p.push_back(point);
        }
        return p;
    }
    
    inline Path densify(const Path &orig, double max_dist)
    {
        return toPath(PathUtils::densify(toPathUtilsPath(orig), max_dist));
    }

    inline Path interpolate(const VictorRightArmConfig &p1, const VictorRightArmConfig &p2, double max_dist)
    {
        Path p{p1, p2};
        return densify(p, max_dist);
    }
}



#endif
