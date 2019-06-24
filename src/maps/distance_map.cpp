#include "maps/distance_map.hpp"


DistanceGrid::DistanceGrid() :
    gpu_voxels::voxelmap::DistanceVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                           VOXEL_SIDE_LENGTH, MT_DISTANCE_VOXELMAP),
    ready_for_operations(false)
{}

void DistanceGrid::insertBox(const Vector3f &corner_min, const Vector3f &corner_max)
{
    float delta = VOXEL_SIDE_LENGTH / 2.0;
    insertPointCloud(geometry_generation::createBoxOfPoints(corner_min, corner_max, delta),
                     eBVM_OCCUPIED);
    ready_for_operations = false;
}

void DistanceGrid::computeDistances()
{
    parallelBanding3D(1, 1, 1, PBA_DEFAULT_M1_BLOCK_SIZE,
                      PBA_DEFAULT_M2_BLOCK_SIZE,
                      PBA_DEFAULT_M3_BLOCK_SIZE, 1);
    ready_for_operations = true;
}

bool DistanceGrid::mergeOccupied(const DenseGrid *other)
{
    ready_for_operations = false;
    return gpu_voxels::voxelmap::DistanceVoxelMap::mergeOccupied(other->getProbVoxelMap());
}

std::pair<Vector3i, DistanceVoxel> DistanceGrid::getClosestObstacle(const DenseGrid *other)
{
    checkReadyForComputation();
    return gpu_voxels::voxelmap::DistanceVoxelMap::getClosestObstacle(other->getProbVoxelMap());
}

DistanceVoxel::pba_dist_t DistanceGrid::getClosestObstacleDistance(const DenseGrid *other)
{
    checkReadyForComputation();
    return gpu_voxels::voxelmap::DistanceVoxelMap::getClosestObstacleDistance(other->getProbVoxelMap());
}


void DistanceGrid::checkReadyForComputation()
{
    if(ready_for_operations)
    {
        return;
    }
    computeDistances();
}
