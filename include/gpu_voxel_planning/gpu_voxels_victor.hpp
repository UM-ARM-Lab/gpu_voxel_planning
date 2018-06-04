#ifndef GPU_VOXELS_VICTOR
#define GPU_VOXELS_VICTOR


#include <gpu_voxels/GpuVoxels.h>
#include "collision_detection.hpp"

#include <arc_utilities/maybe.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


#define VICTOR_ACTUAL_MAP "victor_actual_map"
#define VICTOR_QUERY_MAP "victor_query_map"
#define ENV_MAP "env_map"

#define VICTOR_SWEPT_VOLUME_MAP "victor_swept_volume_map"
#define VICTOR_PATH_ENDPOINTS_MAP "victor_path_endpoints_map"
#define VICTOR_PATH_SOLUTION_MAP "victor_path_solutions_map"
#define OBSTACLE_DISTANCE_MAP "obstacle_distance_map"
#define FULL_MAP "full_map"
#define KNOWN_OBSTACLES_MAP "known_obstacles_map"
#define COMBINED_COLSETS_MAP "combined_colsets_map"


#define SIM_OBSTACLES_MAP "sim_obstacles_map"
#define TMP_MAP "tmp_map"
#define VICTOR_ROBOT "victor_robot"
#define VICTOR_ROBOT_STATIONARY "victor_robot_stationary"


extern std::vector<std::string> SEEN_OBSTACLE_SETS;


typedef robot::JointValueMap VictorConfig;
typedef std::vector<std::vector<double>> Path;


Path densifyPath(const Path &path, int densify_factor);



class GpuVoxelsVictor
{
public:
    GpuVoxelsVictor();

    ~GpuVoxelsVictor();

    void insertVictorIntoMap(const VictorConfig &c, const std::string &map_name);

    void updateActual(const VictorConfig &c);

    void addCollisionPoints(CollisionInformation collision_info);

    /* Sets robot config, adds select links to map, removes swept volume */
    void addCollisionLinks(const VictorConfig &c,
                           const std::vector<std::string> &collision_links,
                           const std::string &map_name);

    void addCollisionSet(const std::vector<VictorConfig> &cs,
                         const std::vector<std::string> &collision_links);
    
    void addQueryLink(const VictorConfig &c, const std::string &link_name);


    
    void resetQuery();

    void addQueryState(const VictorConfig &c);

    size_t countTotalNumCollisions();

    size_t countIntersect(const std::string& map_1, const std::string& map_2);

    void m1_subtract_m2(const std::string& map_1, const std::string& map_2);
    
    std::vector<size_t> countSeenCollisionsInQueryForEach();

    size_t countNumCollisions(const std::string &map_name);

    size_t countTotalNumCollisionsForConfig(const VictorConfig &c);

    std::vector<size_t> seenSizes();

    size_t getNumOccupiedVoxels(const std::string& map_name);
    
    /* Returns true if Victor is not in collision at this config */
    bool queryFreeConfiguration(const VictorConfig &c);

    bool isInJointLimits(const double *values);

    VictorConfig toVictorConfig(const double* values);

    std::vector<double> toValues(VictorConfig config);

    // template<typename T>
    // robot::JointValueMap toRightJointValueMap(const T values);

    
    int determineVictorDist();

    void doVis();
    
    void visPath(const Path &path);
    void hidePath();

public:
    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    int num_observed_sets;
    VictorConfig cur_config;
};




Path densifyPath(const Path &path, int densify_factor);




class SimWorld
{
public:
    SimWorld();
    void initializeObstacles();
    void makeTable();
    void makeSlottedWall();
    bool executePath(const Path &path, size_t &last_index, bool add_col_set);

    bool attemptPath(const Path &path);

    void executeAndReturn(const Path &path);

    Maybe::Maybe<std::string> getCollisionLink(const VictorConfig &c);
    Maybe::Maybe<std::vector<std::string>> getCollisionLinks(const VictorConfig &c);

public:    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    GpuVoxelsVictor victor_model;
};

class RealWorld
{
public:
    RealWorld();
    ~RealWorld();
    bool attemptPath(const Path &path);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void spinUntilUpdate();
    void loadPointCloudFromFile();
    
public:
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    GpuVoxelsVictor victor_model;
    ros::Subscriber joint_sub;
    ros::ServiceClient attempt_path_client;
    ros::ServiceClient get_attempt_status_client;

    bool update_victor_from_messages;
    bool pos_updated;
    int update_all_joint_count{0};
};



#endif
