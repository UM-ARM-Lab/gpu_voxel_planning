#ifndef GPU_VOXELS_VICTOR
#define GPU_VOXELS_VICTOR


#include <gpu_voxels/GpuVoxels.h>
#include "collision_detection.hpp"


std::vector<std::string> SEEN_OBSTACLE_SETS;


typedef robot::JointValueMap VictorConfig;
typedef std::vector<std::vector<double>> Path;

class GpuVoxelsVictor
{
public:
    GpuVoxelsVictor();

    void insertVictorIntoMap(const VictorConfig &c, const std::string &map_name);

    void updateActual(const VictorConfig &c);

    void addCollisionPoints(CollisionInformation collision_info);

    /* Sets robot config, adds select links to map, removes swept volume */
    void addCollisionLinks(const VictorConfig &c,
                           const std::vector<std::string> &collision_links,
                           const std::string &map_name);

    void resetQuery();

    void addQueryState(const VictorConfig &c);

    size_t countNumCollisions();

    size_t countNumCollisions(const VictorConfig &c);

    /* Returns true if Victor is not in collision at this config */
    bool queryFreeConfiguration(const VictorConfig &c);

    bool isInJointLimits(const double *values);

    VictorConfig toVictorConfig(const double* values);

    // template<typename T>
    // robot::JointValueMap toRightJointValueMap(const T values);

    
    int determineVictorDist();

    void doVis();
    void visualizeSolution(const std::vector<VictorConfig> &joint_maps);
    void hideSolution();

public:
    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    int num_observed_sets;
};



class SimWorld
{
public:
    SimWorld();
    void initializeObstacles();
    bool executePath(const Path &path);

public:    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    GpuVoxelsVictor victor_model;

};



#endif
