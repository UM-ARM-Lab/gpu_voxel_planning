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
        DenseGrid grid;
        
        AABB(Vector3f ll, Vector3f ur):
            ll(ll), ur(ur)
        {
            remakeGrid();
        }

        void remakeGrid()
        {
            grid = DenseGrid();
            grid.insertBox(ll, ur);
        }

        void shift(Vector3f dx)
        {
            ll += dx;
            ur += dx;
            remakeGrid();
        }

        void project(DistanceGrid& dg)
        {
            auto result = dg.getClosestObstacle(&grid);
            Vector3i on_self = result.first;
            Vector3i on_distance_grid = Vector3i(result.second.getObstacle());
            Vector3i diff_i = on_distance_grid - on_self;
            Vector3f diff(diff_i.x, diff_i.y, diff_i.z);
            shift(diff*VOXEL_SIDE_LENGTH);
        }
    };
}

#endif
