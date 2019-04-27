#ifndef DISTANCE_MAP_HPP
#define DISTANCE_MAP_HPP

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "maps/prob_map.hpp"

// #define GRID_X_DIM 200
// #define GRID_Y_DIM 200
// #define GRID_Z_DIM 200
// #define VOXEL_SIDE_LENGTH 0.02




/***************************
 *   Distance Grid
 ****************************/

class DistanceGrid : private gpu_voxels::voxelmap::DistanceVoxelMap
{
public:
    DistanceGrid();

    // DistanceGrid(const DistanceGrid &other) :
    //     gpu_voxels::voxelmap::ProbVoxelMap(other.getDimensions(),
    //                                        other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
    // {
    //     copy(&other);
    // }

    // DistanceGrid(const SparseGrid &other) :
    //     gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
    //                                        other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
    // {
    //     merge(&other);
    // }

    // DistanceGrid& operator=(const DistanceGrid &other)
    // {
    //     copy(&other);
    //     return *this;
    // }

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max);

    void computeDistances();

    std::pair<Vector3i, DistanceVoxel> getClosestObstacle(const DenseGrid *other);

    DistanceVoxel::pba_dist_t getClosestObstacleDistance(const DenseGrid *other);

    bool mergeOccupied(const DenseGrid *other);
};


#endif
