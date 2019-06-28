#include "path_utils_addons.hpp"
#include <arc_utilities/path_utils.hpp>

GVP::Path GVP::smooth(GVP::Path path, State &s, double discretization, std::mt19937 &rng)
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
