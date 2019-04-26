#include "maps/prob_map.hpp"

/********************
 ** Sparse Grid
 *******************/
SparseGrid::SparseGrid() : gpu_voxels::voxellist::ProbVoxelList(
    Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
    VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELLIST)
{}

SparseGrid::SparseGrid(const gpu_voxels::voxelmap::ProbVoxelMap &other) :
    gpu_voxels::voxellist::ProbVoxelList(other.getDimensions(),
                                         other.getVoxelSideLength(), MT_PROBAB_VOXELLIST)
{
    merge(&other);
}

SparseGrid& SparseGrid::operator=(const gpu_voxels::voxelmap::ProbVoxelMap &other)
{
    merge(&other);
    return *this;
}




/*******************
 ** Dense Grid
 ******************/
DenseGrid::DenseGrid() : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                                            VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELMAP)
{
}

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

void DenseGrid::copyRandomOccupiedElement(DenseGrid& to) const
{
    size_t num_occupied = countOccupied();
    std::mt19937 generator;
    generator.seed(std::random_device()());
    std::uniform_int_distribution<unsigned long> dist(0, num_occupied);
    unsigned long rand_index = dist(generator);
    copyIthOccupied(&to, rand_index);
}


