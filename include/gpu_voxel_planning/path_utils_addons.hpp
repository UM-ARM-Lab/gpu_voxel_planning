#ifndef PATH_UTILS_ADDONS_HPP
#define PATH_UTILS_ADDONS_HPP

#include "path_utils.hpp"
#include "robot_model.hpp"
#include "state.hpp"
#include <arc_utilities/path_utils.hpp>



namespace GVP
{
    typedef std::vector<VictorRightArmConfig> Path;

    static PathUtils::Path toPathUtilsPath(const Path &p)
    {
        PathUtils::Path pu;
        for(const auto& config: p)
        {
            pu.push_back(config.asVector());
        }
        return pu;
    }

    static Path toPath(const PathUtils::Path &pu)
    {
        Path p;
        for(const auto& point: pu)
        {
            p.push_back(point);
        }
        return p;
    }
    
    static Path densify(const Path &orig, double max_dist)
    {
        return toPath(PathUtils::densify(toPathUtilsPath(orig), max_dist));
    }

    static Path interpolate(const VictorRightArmConfig &p1, const VictorRightArmConfig &p2, double max_dist)
    {
        Path p{p1, p2};
        return densify(p, max_dist);
    }

    
    Path smooth(Path path, State &s, double discretization, std::mt19937 &rng)
    {
        const auto edge_check_fn = [&](const VictorRightArmConfig &q1, const VictorRightArmConfig &q2)
            {
                GVP::Path path = interpolate(q1, q2, discretization);

                auto rng = std::default_random_engine{};
                std::shuffle(std::begin(path), std::end(path), rng);

                for(const auto &config: path)
                {
                    bool valid = s.isPossiblyValid(config);
                    if(!valid)
                    {
                        return false;
                    }
                }
                return true;
            };
        return densify(path_utils::ShortcutSmoothPath(path, 30, 30, 1.0, edge_check_fn, rng), discretization);
    }

}



#endif
