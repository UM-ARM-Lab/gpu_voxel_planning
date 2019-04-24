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

class DistanceGrid : public gpu_voxels::voxelmap::DistanceVoxelMap
{
public:
    DistanceGrid() : gpu_voxels::voxelmap::DistanceVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                                            VOXEL_SIDE_LENGTH, MT_DISTANCE_VOXELMAP){}

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

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max)
    {
        float delta = VOXEL_SIDE_LENGTH / 2.0;
        insertPointCloud(geometry_generation::createBoxOfPoints(corner_min, corner_max, delta),
                         eBVM_OCCUPIED);
    }

    void computeDistances()
    {
        parallelBanding3D(1, 1, 1, PBA_DEFAULT_M1_BLOCK_SIZE,
                          PBA_DEFAULT_M2_BLOCK_SIZE,
                          PBA_DEFAULT_M3_BLOCK_SIZE, 1);
    }

};


#endif
