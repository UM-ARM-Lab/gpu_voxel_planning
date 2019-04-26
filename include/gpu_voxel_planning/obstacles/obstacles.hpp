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

    class Object
    {
    public:
        std::vector<AABB> aabbs;
        DenseGrid occupied;

    public:
        void remakeGrid()
        {
            occupied = DenseGrid();
            for(const auto& aabb: aabbs)
            {
                occupied.add(&aabb.grid);
            }
        }

        void add(const AABB& aabb)
        {
            aabbs.push_back(aabb);
            remakeGrid();
        }

        void shift(Vector3f dx)
        {
            for(auto& aabb: aabbs)
            {
                aabb.shift(dx);
            }
        }

        void project(DistanceGrid& dg)
        {
            throw std::logic_error("Not implemented");
        }
    };

    class ObstacleConfiguration
    {
    public:
        std::vector<Object> obstacles;
        DenseGrid occupied;

    public:
        void remakeGrid()
        {
            occupied = DenseGrid();
            for(const auto& ob: obstacles)
            {
                occupied.add(&ob.occupied);
            }
        }

        void add(const Object& object)
        {
            obstacles.push_back(object);
            remakeGrid();
        }
    };
}

#endif
