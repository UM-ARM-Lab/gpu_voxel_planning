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
        // DenseGrid grid;
        
        AABB(Vector3f ll, Vector3f ur):
            ll(ll), ur(ur)
        {
            // remakeGrid();
        }

        DenseGrid getGrid() const
        {
            DenseGrid grid;
            grid.insertBox(ll, ur);
            return grid;
        }

        void shift(Vector3f dx)
        {
            ll += dx;
            ur += dx;
            // remakeGrid();
        }

        void project(DistanceGrid& dg)
        {
            DenseGrid grid = getGrid();
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
                DenseGrid g = aabb.getGrid();
                occupied.add(&g);
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
            remakeGrid();
        }

        void project(DistanceGrid& dg)
        {
            auto result = dg.getClosestObstacle(&occupied);
            Vector3i on_self = result.first;
            Vector3i on_distance_grid = Vector3i(result.second.getObstacle());
            Vector3i diff_i = on_distance_grid - on_self;
            Vector3f diff(diff_i.x, diff_i.y, diff_i.z);
            // std::cout << "Shifting box by " << diff * VOXEL_SIDE_LENGTH << "\n";
            shift(diff*VOXEL_SIDE_LENGTH);
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

        void project(DistanceGrid& dg)
        {
            if(obstacles.size()==0)
            {
                std::cout << "No projection because no obstacles";
                return;
            }
            
            int closest_ind = -1;
            double closest_dist = std::numeric_limits<double>::infinity();
            for(int i=0; i<obstacles.size(); i++)
            {
                double d = dg.getClosestObstacleDistance(&obstacles[i].occupied);
                if(d < closest_dist)
                {
                    d = closest_dist;
                    closest_ind = i;
                }
            }
            obstacles[closest_ind].project(dg);
            remakeGrid();
        }
    };
}

#endif
