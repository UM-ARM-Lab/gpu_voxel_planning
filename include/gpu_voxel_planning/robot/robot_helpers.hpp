#ifndef GVP_ROBOT_HELPERS_HPP
#define GVP_ROBOT_HELPERS_HPP

#include "gpu_voxel_planning/robot/robot_model.hpp"

namespace GVP
{
    inline size_t getFirstLinkInCollision(Robot& robot, const DenseGrid& true_world)
    {
        auto link_occupancies = robot.getLinkOccupancies();
        size_t first_link_in_collision = 0;
        for(auto &link:link_occupancies)
        {
            if(link.overlapsWith(&true_world))
            {
                break;
            }
            first_link_in_collision++;
        }
        return first_link_in_collision;
    }

    inline DenseGrid sweptVolume(Robot& robot, std::vector<VictorRightArmConfig> path)
    {
        PROFILE_START("ComputeSweptVolume");
        DenseGrid swept_volume;
        for(const auto &config: path)
        {
            PROFILE_START("Config Added to Swept Volume");
            robot.set(config.asMap());
            swept_volume.add(&robot.occupied_space);
            PROFILE_RECORD("Config Added to Swept Volume");
        }

        PROFILE_RECORD("ComputeSweptVolume");
        return swept_volume;
    }
}


#endif
