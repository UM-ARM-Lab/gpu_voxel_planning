#ifndef GPU_PLANNING_OBSTACLES_HPP
#define GPU_PLANNING_OBSTACLES_HPP

#include "maps/prob_map.hpp"
#include "maps/distance_map.hpp"

namespace GVP
{
    class AABB
    {
    public:
        Vector3f ll;
        Vector3f ur;
        
        AABB(Vector3f ll, Vector3f ur):
            ll(ll), ur(ur)
        {}

        DenseGrid toGrid()
        {
            DenseGrid g;
            g.insertBox(ll, ur);
            return g;
        }

        void shift(Vector3f dx)
        {
            ll += dx;
            ur += dx;
        }

        void project(DistanceGrid& dg)
        {
            DenseGrid g = toGrid();
            auto result = dg.getClosestObstacle(&g);
            Vector3i on_self = result.first;
            Vector3i on_distance_grid = Vector3i(result.second.getObstacle());
            Vector3i diff_i = on_distance_grid - on_self;
            Vector3f diff(diff_i.x, diff_i.y, diff_i.z);
            shift(diff*VOXEL_SIDE_LENGTH);
        }
    };
}

#endif
