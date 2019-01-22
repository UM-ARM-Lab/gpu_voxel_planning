#ifndef PROB_MAP_H
#define PROB_MAP_H

#include <gpu_voxels/GpuVoxels.h>


class VoxelGrid
{
public:
    
};

class DenseGrid;


class SparseGrid: public gpu_voxels::voxellist::ProbVoxelList
{
public:
    SparseGrid();
    SparseGrid(const DenseGrid &other);
};


class DenseGrid : public gpu_voxels::voxelmap::ProbVoxelMap
{
public:
    DenseGrid();
    
    DenseGrid(const DenseGrid &other);

    DenseGrid(const SparseGrid &other);

    DenseGrid& operator=(const DenseGrid &other);

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max);

};



#endif
