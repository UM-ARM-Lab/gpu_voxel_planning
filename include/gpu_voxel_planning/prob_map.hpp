#ifndef PROB_MAP_HPP
#define PROB_MAP_HPP

#include "prob_map.h"
#include <gpu_voxels/helpers/GeometryGeneration.h>

#define GRID_X_DIM 200
#define GRID_Y_DIM 200
#define GRID_Z_DIM 200
#define VOXEL_SIDE_LENGTH 0.02


/***************************
 *   Sparse Grid
 ****************************/

SparseGrid::SparseGrid():
    gpu_voxels::voxellist::ProbVoxelList(
        Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
        VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELLIST)
{
}

SparseGrid::SparseGrid(const DenseGrid &other) :
    gpu_voxels::voxellist::ProbVoxelList(other.getDimensions(),
                                         other.getVoxelSideLength(), MT_PROBAB_VOXELLIST)
{
    merge(&other);
}





/***************************
 *   Dense Grid
 ****************************/

DenseGrid::DenseGrid() : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                                            VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELMAP){}

DenseGrid::DenseGrid(const DenseGrid &other) :
    gpu_voxels::voxelmap::ProbVoxelMap(other.getDimensions(),
                                       other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
{
    copy(&other);
}

DenseGrid::DenseGrid(const SparseGrid &other) :
    gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                       other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
{
    merge(&other);
}

DenseGrid& DenseGrid::operator=(const DenseGrid &other)
{
    copy(&other);
    return *this;
}

void DenseGrid::insertBox(const Vector3f &corner_min, const Vector3f &corner_max)
{
    float delta = VOXEL_SIDE_LENGTH / 2.0;
    insertPointCloud(geometry_generation::createBoxOfPoints(corner_min, corner_max, delta),
                     eBVM_OCCUPIED);
}





#endif
