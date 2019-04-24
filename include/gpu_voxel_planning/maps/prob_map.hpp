#ifndef PROB_MAP_HPP
#define PROB_MAP_HPP

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>

#define GRID_X_DIM 256
#define GRID_Y_DIM 256
#define GRID_Z_DIM 256
#define VOXEL_SIDE_LENGTH 0.02


class VoxelGrid
{
public:
    
};


/***************************
 *   Sparse Grid
 ****************************/
class SparseGrid: public gpu_voxels::voxellist::ProbVoxelList
{
public:
    SparseGrid():
        gpu_voxels::voxellist::ProbVoxelList(
            Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
            VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELLIST)
    {}

    SparseGrid(const gpu_voxels::voxelmap::ProbVoxelMap &other) :
        gpu_voxels::voxellist::ProbVoxelList(other.getDimensions(),
                                             other.getVoxelSideLength(), MT_PROBAB_VOXELLIST)
    {
        merge(&other);
    }

    SparseGrid& operator=(const gpu_voxels::voxelmap::ProbVoxelMap &other)
    {
        merge(&other);
        return *this;
    }
};




/***************************
 *   Dense Grid
 ****************************/

class DenseGrid : public gpu_voxels::voxelmap::ProbVoxelMap
{
public:
    DenseGrid() : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                                                VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELMAP){}

    DenseGrid(const DenseGrid &other) :
        gpu_voxels::voxelmap::ProbVoxelMap(other.getDimensions(),
                                           other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
    {
        copy(&other);
    }

    DenseGrid(const SparseGrid &other) :
        gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                           other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
    {
        merge(&other);
    }

    DenseGrid& operator=(const DenseGrid &other)
    {
        copy(&other);
        return *this;
    }

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max)
    {
        float delta = VOXEL_SIDE_LENGTH / 2.0;
        insertPointCloud(geometry_generation::createBoxOfPoints(corner_min, corner_max, delta),
                         eBVM_OCCUPIED);
    }

};


#endif
