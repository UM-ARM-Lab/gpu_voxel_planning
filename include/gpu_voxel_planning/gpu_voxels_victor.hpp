#ifndef GPU_VOXELS_VICTOR
#define GPU_VOXELS_VICTOR


#include <gpu_voxels/GpuVoxels.h>
#include "collision_detection.hpp"


class GpuVoxelsVictor
{
public:
    GpuVoxelsVictor();

    void updateVictorPosition(robot::JointValueMap joint_positions);

    void addCollisionPoints(CollisionInformation collision_info);

    size_t countNumCollisions(const robot::JointValueMap &joint_values_map);

    /* Returns true if Victor is not in collision at this config */
    bool queryFreeConfiguration(const robot::JointValueMap &joint_values_map);

    bool isInJointLimits(const double *values);

    robot::JointValueMap toRightJointValueMap(const double* values);

    // template<typename T>
    // robot::JointValueMap toRightJointValueMap(const T values);

    
    int determineVictorDist();

    void doVis();
    void visualizeSolution(const std::vector<robot::JointValueMap> &joint_maps);
    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
};




#endif
