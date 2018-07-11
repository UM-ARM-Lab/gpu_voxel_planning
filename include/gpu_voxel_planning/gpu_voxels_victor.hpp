#ifndef GPU_VOXELS_VICTOR_HPP
#define GPU_VOXELS_VICTOR_HPP


#include <gpu_voxels/GpuVoxels.h>
// #include "collision_detection.hpp"

#include <arc_utilities/maybe.hpp>
#include "path_utils.hpp"
#include <ros/ros.h>

#include <bitset>



#define VICTOR_ACTUAL_MAP "victor_actual_map"
#define VICTOR_QUERY_MAP "victor_query_map"
#define ENV_MAP "env_map"
#define SAMPLED_WORLD_MAP "sampled_world_map"

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


extern std::vector<std::string> COLLISION_HYPOTHESIS_SETS;
extern std::vector<std::string> HCHS; //hypothetical collision hypothesis sets
#define HFREE "hypothetical_free"
#define HCHS_NEW "new_hypothetical_chs"


typedef robot::JointValueMap VictorConfig;


template<size_t N>
std::vector<std::bitset<N>> allBinaryPossibilities()
{
    const long len = (long)N;
    std::vector<std::bitset<N>> all;
    all.reserve(std::pow(2, len));
    for(long i=0; i<std::pow(2, len); i++)
    {
        all.push_back((std::bitset<N>)i);
    }
    return all;
}




class GpuVoxelsVictor
{
public:
    GpuVoxelsVictor();

    ~GpuVoxelsVictor();

    void insertVictorIntoMap(const VictorConfig &c, const std::string &map_name);

    void updateActual(const VictorConfig &c);

    /* Sets robot config, adds select links to map, removes swept volume */
    void addLinks(const VictorConfig &config,
                  const std::vector<std::string> &link_names,
                  const std::string &map_name);

    void addCHS(const Path &path,
                const std::vector<std::string> &collision_links);

    void addCHSToMap(const Path &path,
                     const std::vector<std::string> &collision_links,
                     const std::string &map);

    void resetHypothetical();
    
    void addQueryLink(const VictorConfig &c, const std::string &link_name);

    void resetQuery();

    void addQueryState(const VictorConfig &c);

    size_t countTotalCHSCollisions();

    size_t countIntersect(const std::string& map_1, const std::string& map_2);
    
    bool overlaps(const std::string& map_1, const std::string& map_2);

    voxelmap::ProbVoxelMap* getMap(const std::string& map_name)
        {
            return gvl->getMap(map_name)->as<voxelmap::ProbVoxelMap>();
        };

    void removeSweptVolume(const std::string& map_name);
    
    std::vector<size_t> countCHSCollisions();

    size_t countNumCollisions(const std::string &map_name);

    size_t countTotalCHSCollisionsForConfig(const VictorConfig &c);

    std::vector<size_t> chsSizes();

    size_t countVoxels(const std::string& map_name);
    
    /* Returns true if Victor is not in collision at this config */
    bool queryFreeConfiguration(const VictorConfig &c);

    bool isInJointLimits(const double *values);

    VictorConfig toVictorConfig(const double* values);

    std::vector<double> toValues(VictorConfig config);

    int determineVictorDist();

    void doVis();
    
    void visPath(const Path &path);
    void hidePath();

    // double calc_prob_chss(double p_occ, std::vector<std::string> chs_maps);


    // double calc_prob_chss_and_freespace(double p_occ, std::vector<std::string> chs_maps,
    //                                     std::string freespace_map_name);

    

        

public:

    const std::vector<std::string> right_arm_joint_names;
    std::vector<std::string> right_arm_collision_link_names;
    std::vector<std::string> right_gripper_collision_link_names;
    const std::vector<std::string> left_arm_joint_names;
    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    int num_observed_chs;
    VictorConfig cur_config;
};

#endif
