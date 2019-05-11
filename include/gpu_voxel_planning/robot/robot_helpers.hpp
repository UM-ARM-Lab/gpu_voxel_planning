#ifndef GVP_ROBOT_HELPERS_HPP
#define GVP_ROBOT_HELPERS_HPP

#include "robot_model.hpp"

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
}


#endif
