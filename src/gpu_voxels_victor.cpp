#include "gpu_voxels_victor.hpp"
#include "common_names.hpp"
#include "hardcoded_params.h"

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <arc_utilities/arc_helpers.hpp>
#include <math.h>


std::vector<std::string> COLLISION_HYPOTHESIS_SETS;
std::vector<std::string> HCHS;


#define NUM_SETS 100


std::vector<std::string> i_right_arm_joint_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2",
        "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5",
        "victor_right_arm_joint_6", "victor_right_arm_joint_7"};

std::vector<std::string> i_right_arm_collision_link_names{
    "victor_right_arm_link_3",
        "victor_right_arm_link_4",
        "victor_right_arm_link_5",
        "victor_right_arm_link_6",
        "victor_right_arm_link_7",
        "victor_right_gripper_palm",
        "victor_right_gripper_mounting_bracket",
        "victor_right_gripper_fingerA_base", "victor_right_gripper_fingerA_dist",
        "victor_right_gripper_fingerA_med", "victor_right_gripper_fingerA_prox",
        "victor_right_gripper_fingerB_base", "victor_right_gripper_fingerB_dist",
        "victor_right_gripper_fingerB_med", "victor_right_gripper_fingerB_prox",
        "victor_right_gripper_fingerC_base", "victor_right_gripper_fingerC_dist",
        "victor_right_gripper_fingerC_med", "victor_right_gripper_fingerC_prox"
        };

std::vector<std::string> i_right_gripper_collision_link_names{
    // "victor_right_arm_link_6",
    "victor_right_arm_link_7",
        "victor_right_gripper_palm",
        "victor_right_gripper_mounting_bracket",
        "victor_right_gripper_fingerA_base", "victor_right_gripper_fingerA_dist",
        "victor_right_gripper_fingerA_med", "victor_right_gripper_fingerA_prox",
        "victor_right_gripper_fingerB_base", "victor_right_gripper_fingerB_dist",
        "victor_right_gripper_fingerB_med", "victor_right_gripper_fingerB_prox",
        "victor_right_gripper_fingerC_base", "victor_right_gripper_fingerC_dist",
        "victor_right_gripper_fingerC_med", "victor_right_gripper_fingerC_prox"

        };

std::vector<std::string> i_left_arm_joint_names{"victor_left_arm_joint_1", "victor_left_arm_joint_2", "victor_left_arm_joint_3", "victor_left_arm_joint_4", "victor_left_arm_joint_5", "victor_left_arm_joint_6", "victor_left_arm_joint_7"};



GpuVoxelsVictor::GpuVoxelsVictor():
    num_observed_sets(0),
    right_arm_joint_names(i_right_arm_joint_names),
    right_arm_collision_link_names(i_right_arm_collision_link_names),
    right_gripper_collision_link_names(i_right_gripper_collision_link_names),
    left_arm_joint_names(i_left_arm_joint_names)
{
    std::cout << "Initializing gpu voxels victor\n";
    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(200, 200, 200, 0.02);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_QUERY_MAP); //map for queries on victor validity
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_ACTUAL_MAP); //map for victors current state
    gvl->addMap(MT_PROBAB_VOXELMAP, ENV_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, TMP_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, KNOWN_OBSTACLES_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, COMBINED_COLSETS_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, SIM_OBSTACLES_MAP);
    gvl->addMap(MT_BITVECTOR_VOXELLIST, VICTOR_PATH_SOLUTION_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_SWEPT_VOLUME_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_PATH_ENDPOINTS_MAP);
    gvl->addMap(MT_DISTANCE_VOXELMAP, OBSTACLE_DISTANCE_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, FULL_MAP);

    gvl->addMap(MT_PROBAB_VOXELMAP, HFREE);
    gvl->addMap(MT_PROBAB_VOXELMAP, HCHS_NEW);


    gvl->insertBoxIntoMap(Vector3f(-1,-1,-1), Vector3f(300*0.02,300*0.02,300*0.02), FULL_MAP, PROB_OCCUPIED);

    if(PEG_IN_HOLE || REAL_ROBOT)
    {
        gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_with_rod.urdf", false);
        right_gripper_collision_link_names.push_back("rod");
        right_arm_collision_link_names.push_back("rod");
    }
    else{
        gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_only.urdf", false);
    }

    // gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor.urdf", false);

    gvl->addRobot(VICTOR_ROBOT_STATIONARY, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_left_arm_and_body.urdf", false);


    
    COLLISION_HYPOTHESIS_SETS.resize(NUM_SETS);
    HCHS.resize(NUM_SETS);
    for(int i=0; i < NUM_SETS; i++)
    {
        COLLISION_HYPOTHESIS_SETS[i] = "chs_" + std::to_string(i);
        gvl->addMap(MT_PROBAB_VOXELMAP, COLLISION_HYPOTHESIS_SETS[i]);
        gvl->visualizeMap(COLLISION_HYPOTHESIS_SETS[i]);

        HCHS[i] = "hypothtical_chs_" + std::to_string(i);
        gvl->addMap(MT_PROBAB_VOXELMAP, HCHS[i]);
                
    }
    gvl->visualizeMap(VICTOR_ACTUAL_MAP);
    gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP);

    if(VIDEO_VISUALIZE)
    {
        gvl->visualizeMap(VICTOR_QUERY_MAP);
    }
}


GpuVoxelsVictor::~GpuVoxelsVictor()
{
    std::cout << "Destructor for gpu voxels victor...";
    gvl.reset();
    std::cout << "...gvl reset successful\n";
}

void GpuVoxelsVictor::removeSweptVolume(const std::string& map_name)
{
    getMap(map_name)->subtract(getMap(VICTOR_SWEPT_VOLUME_MAP));
}

void GpuVoxelsVictor::insertVictorIntoMap(const VictorConfig &c, const std::string &map_name)
{
    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, map_name, PROB_OCCUPIED);
}


void GpuVoxelsVictor::updateActual(const VictorConfig &c)
{
    cur_config = c;

    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    gvl->clearMap(VICTOR_ACTUAL_MAP);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_SWEPT_VOLUME_MAP, PROB_OCCUPIED);

    for(size_t i=0; i<num_observed_sets; i++)
    {
        removeSweptVolume(COLLISION_HYPOTHESIS_SETS[i]);
    }
    removeSweptVolume(COMBINED_COLSETS_MAP);
}


void GpuVoxelsVictor::resetHypothetical()
{
    for(size_t i=0; i < num_observed_sets; i++)
    {
        getMap(HCHS[i])->copy(getMap(COLLISION_HYPOTHESIS_SETS[i]));
    }
    getMap(HFREE)->copy(getMap(VICTOR_SWEPT_VOLUME_MAP));
    gvl->clearMap(HCHS_NEW);
}

void GpuVoxelsVictor::resetQuery()
{
    gvl->clearMap(VICTOR_QUERY_MAP);
}

void GpuVoxelsVictor::addQueryState(const VictorConfig &c)
{
    insertVictorIntoMap(c, VICTOR_QUERY_MAP);
}

/*
 *  Count collisions between query and env maps
 *  addQueryState should probably be run at least once first
 */
size_t GpuVoxelsVictor::countTotalCHSCollisions()
{
    size_t total_col = 0;
    for(auto cols: countCHSCollisions())
    {
        total_col += cols;
    }
    return total_col;
}


std::vector<size_t> GpuVoxelsVictor::countCHSCollisions()
{
    PROFILE_START("Chs sizes, robot intersection")
    std::vector<size_t> collisions_in_chs;
    for(int i=0; i<num_observed_sets; i++)
    {
        collisions_in_chs.push_back(countNumCollisions(COLLISION_HYPOTHESIS_SETS[i]));
    }
    PROFILE_RECORD("Chs sizes, robot intersection")
    return collisions_in_chs;
}

/*
 *  Count collisions between query and a particular map
 *  addQueryState should probably be run at least once first
 */
size_t GpuVoxelsVictor::countNumCollisions(const std::string &map_name)
{
    return countIntersect(VICTOR_QUERY_MAP, map_name);
}


size_t GpuVoxelsVictor::countTotalCHSCollisionsForConfig(const VictorConfig &c)
{
    PROFILE_START(ISVALID_INSERTION);
    PROFILE_START(QUERY_INSERTION);
    
    resetQuery();
    addQueryState(c);

    PROFILE_RECORD(ISVALID_INSERTION);
    PROFILE_RECORD(QUERY_INSERTION);
    return countTotalCHSCollisions();
}

/*
 *  Returns the number of voxels in each chs collision map
 */
std::vector<size_t> GpuVoxelsVictor::chsSizes()
{
    std::vector<size_t> chs_sizes;
    chs_sizes.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        chs_sizes[i] = countVoxels(COLLISION_HYPOTHESIS_SETS[i]);
    }
    return chs_sizes;
}

size_t GpuVoxelsVictor::countVoxels(const std::string& map_name)
{
    return countIntersect(FULL_MAP, map_name);
}


bool GpuVoxelsVictor::queryFreeConfiguration(const VictorConfig &c)
{
    return countTotalCHSCollisionsForConfig(c) == 0;
}


size_t GpuVoxelsVictor::countIntersect(const std::string& map_1, const std::string& map_2)
{
    return getMap(map_1)->collideWith(getMap(map_2));
}


void GpuVoxelsVictor::addQueryLink(const VictorConfig &c,
                                   const std::string &link_name)
{
    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    RobotInterfaceSharedPtr rob = gvl->getRobot(VICTOR_ROBOT);
    const MetaPointCloud* clouds = rob->getTransformedClouds();
    rob->syncToHost();

    int16_t cloud_num = clouds->getCloudNumber(link_name);
    uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
    const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
    const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
    gvl->insertPointCloudIntoMap(cloud, VICTOR_QUERY_MAP, PROB_OCCUPIED);
}


void GpuVoxelsVictor::addLinks(const VictorConfig &config,
                               const std::vector<std::string> &link_names,
                               const std::string &map_name)
{
    gvl->setRobotConfiguration(VICTOR_ROBOT, config);
    RobotInterfaceSharedPtr rob = gvl->getRobot(VICTOR_ROBOT);
    const MetaPointCloud* clouds = rob->getTransformedClouds();
    // clouds->syncToHost();
    rob->syncToHost();

    for(auto link_name: link_names)
    {
        int16_t cloud_num = clouds->getCloudNumber(link_name);
        uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
        // std::cout << link_name << " has " << cloud_size << " points \n";
        const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
        const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
        gvl->insertPointCloudIntoMap(cloud, map_name, PROB_OCCUPIED);
    }
    removeSweptVolume(map_name);
}

void GpuVoxelsVictor::addCHS(const std::vector<VictorConfig> &cs,
                             const std::vector<std::string> &collision_links)
{
    if(collision_links.size() == 0)
    {
        std::cout << "Attempted to add a collision set with no links, exiting\n";
        assert(false);
        return;
    }
    if(cs.size() == 0)
    {
        std::cout << "Attempted to add a collision set with no configurations, exiting\n";
        assert(false);
        return;
    }
    for(auto &c: cs)
    {
        addLinks(c, collision_links, COLLISION_HYPOTHESIS_SETS[num_observed_sets]);
        addLinks(c, collision_links, COMBINED_COLSETS_MAP);
    }
    gvl->visualizeMap(COLLISION_HYPOTHESIS_SETS[num_observed_sets]);
    num_observed_sets++;

    if(num_observed_sets >= NUM_SETS - 3)
    {
        std::cout << "Getting close to set limit\n";
    }
           
    if(num_observed_sets >= NUM_SETS)
    {
        std::cout << "Set limit reached!. Everything now going in last set\n";
        num_observed_sets--;
    }
    
    for(int i=0; i<num_observed_sets; i++)
    {
        removeSweptVolume(COLLISION_HYPOTHESIS_SETS[i]);
    }
    removeSweptVolume(COMBINED_COLSETS_MAP);
}


/**
 *  Return the number of cells victor is from the nearest obstacle
 */
int GpuVoxelsVictor::determineVictorDist()
{
    boost::shared_ptr<voxelmap::ProbVoxelMap> victor = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap(VICTOR_ACTUAL_MAP));

    boost::shared_ptr<voxelmap::DistanceVoxelMap> dist_map = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap(OBSTACLE_DISTANCE_MAP));
    
    // std::cout << "closest victor dist: " << dist_map->getClosestObstacleDistance(victor) << "\n";
    return int(dist_map->getClosestObstacleDistance(victor));
}


void GpuVoxelsVictor::doVis()
{
    gvl->visualizeMap(VICTOR_ACTUAL_MAP, true);
    for(int i=0; i<num_observed_sets; i++)
        gvl->visualizeMap(COLLISION_HYPOTHESIS_SETS[i]);

}


void GpuVoxelsVictor::visPath(const Path &path)
{
    gvl->clearMap(VICTOR_PATH_SOLUTION_MAP);
    std::cout << "Visualizing path\n";

    // Some new voxels seem to be needed to force the map to refresh in the visualizer

    for(size_t step = 0; step < path.size(); step++)
    {
        PROFILE_START(INSERT_VIZ_SOLUTION);
        auto config = toVictorConfig(path[step].data());
        // update the robot joints:
        gvl->setRobotConfiguration(VICTOR_ROBOT, config);
        // insert the robot into the map:
        gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_SOLUTION_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
        PROFILE_RECORD(INSERT_VIZ_SOLUTION);
    }

    gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP, true);
}


void GpuVoxelsVictor::hidePath()
{
    visPath(Path());
}




/*
 *  Checks if values are in joint limits. Currently assuems values is a list of right joint values (length 7).
 */
bool GpuVoxelsVictor::isInJointLimits(const double *values)
{
    // TODO: Remove hardcoded joint limits:
    std::vector<double> right_joint_lower_deg = {-170, -120, -170, -120, -170, -120, -175};
    std::vector<double> right_joint_upper_deg = {170, 120,  170,  120,  170,  120,  175};

    for(int i=0; i<7; i++)
    {
        if (values[i] < (right_joint_lower_deg[i] * 3.1415/180)){
            return false;
        }
        if (values[i] > (right_joint_upper_deg[i] * 3.1415/180)){
            return false;
        }
    }
    return true;
}


VictorConfig GpuVoxelsVictor::toVictorConfig(const double* values)
{
    VictorConfig jvm;
    for(size_t i=0; i<right_arm_joint_names.size(); i++)
    {
        jvm[right_arm_joint_names[i]] = values[i];
    }
    return jvm;
}

std::vector<double> GpuVoxelsVictor::toValues(VictorConfig config)
{
    std::vector<double> values;
    for(auto &joint_name: right_arm_joint_names)
    {
        values.push_back(config[joint_name]);
    }
    return values;
}






