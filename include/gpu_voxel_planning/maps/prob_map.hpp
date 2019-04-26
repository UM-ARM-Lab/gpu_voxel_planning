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
    SparseGrid();

    SparseGrid(const gpu_voxels::voxelmap::ProbVoxelMap &other);

    SparseGrid& operator=(const gpu_voxels::voxelmap::ProbVoxelMap &other);
};




/***************************
 *   Dense Grid
 ****************************/

class DenseGrid : public gpu_voxels::voxelmap::ProbVoxelMap
{
public:
    DenseGrid();
    
    DenseGrid(const DenseGrid &other);

    DenseGrid(const SparseGrid &other);

    DenseGrid& operator=(const DenseGrid &other);

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max);

    void copyRandomOccupiedElement(DenseGrid& to) const;
};


#endif
