#ifndef PROB_MAP_HPP
#define PROB_MAP_HPP

#include <gpu_voxels/GpuVoxels.h>
#include <visualization_msgs/Marker.h>

#define GRID_X_DIM 200
#define GRID_Y_DIM 200
#define GRID_Z_DIM 200
#define VOXEL_SIDE_LENGTH 0.02


class ProbGrid : public gpu_voxels::voxelmap::ProbVoxelMap
{
public:
    ProbGrid() : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM),
                                                    VOXEL_SIDE_LENGTH, MT_PROBAB_VOXELMAP){};
    ProbGrid(const ProbGrid &other) :
        gpu_voxels::voxelmap::ProbVoxelMap(other.getDimensions(),
                                           other.getVoxelSideLength(), MT_PROBAB_VOXELMAP)
    {
        copy(&other);
    }

    ProbGrid& operator=(const ProbGrid &other)
    {
        copy(&other);
        return *this;
    }
};





#endif
