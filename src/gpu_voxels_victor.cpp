#include "gpu_voxels_victor.hpp"
#include "common_names.hpp"
#include "hardcoded_params.h"
#include <gpu_voxel_planning/CollisionInformation.h>
#include <gpu_voxel_planning/AttemptPathStart.h>
#include <gpu_voxel_planning/AttemptPathResult.h>

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <arc_utilities/arc_helpers.hpp>
#include <arm_pointcloud_utilities/load_save_to_file.h>


#include <boost/thread/lock_guard.hpp>
#include <boost/thread/locks.hpp>
#include <mutex>
#include <math.h>


using namespace arm_pointcloud_utilities;

std::vector<std::string> SEEN_OBSTACLE_SETS;


// #define PROB_OCCUPIED BitVoxelMeaning(255)
#define PROB_OCCUPIED eBVM_OCCUPIED
#define PROB_FREE eBVM_FREE

#define NUM_SETS 100


std::vector<std::string> right_arm_joint_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2",
        "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5",
        "victor_right_arm_joint_6", "victor_right_arm_joint_7"};

std::vector<std::string> right_arm_collision_link_names{
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

std::vector<std::string> victor_right_gripper_collision_names{
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

std::vector<std::string> left_arm_joint_names{"victor_left_arm_joint_1", "victor_left_arm_joint_2", "victor_left_arm_joint_3", "victor_left_arm_joint_4", "victor_left_arm_joint_5", "victor_left_arm_joint_6", "victor_left_arm_joint_7"};



GpuVoxelsVictor::GpuVoxelsVictor():
    num_observed_sets(0)
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
    // gvl->addMap(MT_BITVECTOR_VOXELMAP, ENV_MAP);
    gvl->addMap(MT_BITVECTOR_VOXELLIST, VICTOR_PATH_SOLUTION_MAP);
    // gvl->addMap(MT_BITVECTOR_VOXELMAP, ENV_MAP_RED);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_SWEPT_VOLUME_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_PATH_ENDPOINTS_MAP);
    gvl->addMap(MT_DISTANCE_VOXELMAP, OBSTACLE_DISTANCE_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, FULL_MAP);

    gvl->insertBoxIntoMap(Vector3f(-1,-1,-1), Vector3f(300*0.02,300*0.02,300*0.02), FULL_MAP, PROB_OCCUPIED);

    if(PEG_IN_HOLE || REAL_ROBOT)
    {
        gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_with_rod.urdf", false);
        victor_right_gripper_collision_names.push_back("rod");
        right_arm_collision_link_names.push_back("rod");
    }
    else{
        gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_only.urdf", false);
    }

    // gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor.urdf", false);

    gvl->addRobot(VICTOR_ROBOT_STATIONARY, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_left_arm_and_body.urdf", false);

    VictorConfig left_arm_config;

    std::vector<double> left_arm_joint_values = {1.57, 1.57, 0, 0, 0, 0 ,0};
    // std::vector<double> left_arm_joint_values = {-1.417, 1.566, -1.151, 1.293, 2.437, 1.406, -1.12};
    
    for(size_t i=0; i<left_arm_joint_values.size(); i++)
    {
        left_arm_config[left_arm_joint_names[i]] = left_arm_joint_values[i];
    }

    if(true != REAL_ROBOT)
    {
        gvl->setRobotConfiguration(VICTOR_ROBOT_STATIONARY, left_arm_config);
        gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED);

        gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, SIM_OBSTACLES_MAP, PROB_OCCUPIED);
    }



    VictorConfig right_gripper_config;
    if(REAL_ROBOT)
    {}
    else{
        right_gripper_config["victor_right_gripper_fingerA_joint_2"] = 1.5;
        right_gripper_config["victor_right_gripper_fingerB_joint_2"] = 1.5;
        right_gripper_config["victor_right_gripper_fingerC_joint_2"] = 1.5;
        gvl->setRobotConfiguration(VICTOR_ROBOT, right_gripper_config);
    }
    
    SEEN_OBSTACLE_SETS.resize(NUM_SETS);
    for(int i=0; i < NUM_SETS; i++)
    {
        SEEN_OBSTACLE_SETS[i] = "seen_obstacles_" + std::to_string(i);
        gvl->addMap(MT_PROBAB_VOXELMAP, SEEN_OBSTACLE_SETS[i]);
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[i]);
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
    std::cout << "Destructor for gpu voxels victor\n";
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
        m1_subtract_m2(SEEN_OBSTACLE_SETS[i], VICTOR_SWEPT_VOLUME_MAP);
    }
    m1_subtract_m2(COMBINED_COLSETS_MAP, VICTOR_SWEPT_VOLUME_MAP);
}


void GpuVoxelsVictor::m1_subtract_m2(const std::string& map_1, const std::string& map_2)
{
    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(map_1);
    voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
    obstacles->subtract(gvl->getMap(map_2)->as<voxelmap::ProbVoxelMap>());

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
size_t GpuVoxelsVictor::countTotalNumCollisions()
{
    size_t total_col = 0;
    for(auto cols: countSeenCollisionsInQueryForEach())
    {
        total_col += cols;
    }
    return total_col;
}


std::vector<size_t> GpuVoxelsVictor::countSeenCollisionsInQueryForEach()
{
    PROFILE_START("Seen sizes, robot intersection")
    std::vector<size_t> collisions_in_seen;
    for(int i=0; i<num_observed_sets; i++)
    {
        collisions_in_seen.push_back(countNumCollisions(SEEN_OBSTACLE_SETS[i]));
    }
    PROFILE_RECORD("Seen sizes, robot intersection")
    return collisions_in_seen;
}

/*
 *  Count collisions between query and a particular map
 *  addQueryState should probably be run at least once first
 */
size_t GpuVoxelsVictor::countNumCollisions(const std::string &map_name)
{
    return countIntersect(VICTOR_QUERY_MAP, map_name);
}


size_t GpuVoxelsVictor::countTotalNumCollisionsForConfig(const VictorConfig &c)
{
    PROFILE_START(ISVALID_INSERTION);
    PROFILE_START(QUERY_INSERTION);
    
    resetQuery();

    addQueryState(c);

    PROFILE_RECORD(ISVALID_INSERTION);
    PROFILE_RECORD(QUERY_INSERTION);
    return countTotalNumCollisions();
}

/*
 *  Returns the number of voxels in each seen collision map
 */
std::vector<size_t> GpuVoxelsVictor::seenSizes()
{
    std::vector<size_t> seen_sizes;
    seen_sizes.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        seen_sizes[i] = getNumOccupiedVoxels(SEEN_OBSTACLE_SETS[i]);
    }
    return seen_sizes;
}

size_t GpuVoxelsVictor::getNumOccupiedVoxels(const std::string& map_name)
{
    return countIntersect(FULL_MAP, map_name);
}


bool GpuVoxelsVictor::queryFreeConfiguration(const VictorConfig &c)
{
    return countTotalNumCollisionsForConfig(c) == 0;
}


size_t GpuVoxelsVictor::countIntersect(const std::string& map_1, const std::string& map_2)
{
    return gvl->getMap(map_1)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(map_2)->as<voxelmap::ProbVoxelMap>());
}






// void GpuVoxelsVictor::addCollisionPoints(CollisionInformation collision_info)
// {
//     std::lock_guard<boost::recursive_timed_mutex> g(gvl->getMap(VICTOR_ACTUAL_MAP)->m_mutex);
//     if(!collision_info.collision)
//     {
//         std::cout << "Asked to add collision, but provided CollisionInformation indicates no collision\n";
//         return;
//     }
//     VictorConfig cur_joints, extended_joints;
//     for(size_t i = 0; i < collision_info.joints.size(); i++)
//     {
//         cur_joints[right_arm_joint_names[i]] = collision_info.joints[i];
//         extended_joints[right_arm_joint_names[i]] = collision_info.joints[i] + 0.05 * collision_info.dirs[i];
//     }

//     gvl->setRobotConfiguration(VICTOR_ROBOT, cur_joints);

//     gvl->clearMap(VICTOR_ACTUAL_MAP);
//     gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    
//     int dist = determineVictorDist();
//     // if(dist <= 1)
//     // {
//     //     std::cout << "Already knew victor would be in collision. Skipping\n";
//     //     return;
//     // }
            

//     // Update robot to be slightly into collision object.
//     // Insert only the last few links (heuristic to avoid adding too many points)


//     addCollisionLinks(extended_joints, collision_info.links_in_contact, ENV_MAP);
// }

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


void GpuVoxelsVictor::addCollisionLinks(const VictorConfig &c,
                                        const std::vector<std::string> &collision_links,
                                        const std::string &map_name)
{
    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    RobotInterfaceSharedPtr rob = gvl->getRobot(VICTOR_ROBOT);
    const MetaPointCloud* clouds = rob->getTransformedClouds();
    // clouds->syncToHost();
    rob->syncToHost();

    for(auto collision_link_name: collision_links)
    {
        int16_t cloud_num = clouds->getCloudNumber(collision_link_name);
        uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
        // std::cout << collision_link_name << " has " << cloud_size << " points \n";
        const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
        const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
        gvl->insertPointCloudIntoMap(cloud, map_name, PROB_OCCUPIED);
    }

    m1_subtract_m2(map_name, VICTOR_SWEPT_VOLUME_MAP);
}

void GpuVoxelsVictor::addCollisionSet(const std::vector<VictorConfig> &cs,
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
        addCollisionLinks(c, collision_links, SEEN_OBSTACLE_SETS[num_observed_sets]);
        addCollisionLinks(c, collision_links, COMBINED_COLSETS_MAP);
    }
    gvl->visualizeMap(SEEN_OBSTACLE_SETS[num_observed_sets]);
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
    
    for(size_t i=0; i<num_observed_sets; i++)
    {
        m1_subtract_m2(SEEN_OBSTACLE_SETS[i], VICTOR_SWEPT_VOLUME_MAP);
    }

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
    for(size_t i=0; i<num_observed_sets; i++)
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[i]);

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



// void VictorValidator::insertStartAndGoal(const ob::ScopedState<> &start, const ob::ScopedState<> &goal) const
// {

//     gvl->clearMap(VICTOR_PATH_ENDPOINTS_MAP);
//     VictorConfig state_joint_values = toVictorConfig<ob::ScopedState<>>(start);

//     // update the robot joints:
//     gvl->setRobotConfiguration(VICTOR_ROBOT, state_joint_values);
//     gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_ENDPOINTS_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

//     state_joint_values = toVictorConfig<ob::ScopedState<>>(goal);

//     // update the robot joints:
//     gvl->setRobotConfiguration(VICTOR_ROBOT, state_joint_values);
//     gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_ENDPOINTS_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));

// }



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






/*************************************
 **       SIM OBSTACLES             **
 ************************************/
SimWorld::SimWorld()
{
    std::cout << "Creating sim world\n";
    gvl = gpu_voxels::GpuVoxels::getInstance();

    // if(USE_KNOWN_OBSTACLES)
    // {
        gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
    // }
    gvl->visualizeMap(SIM_OBSTACLES_MAP);


    double init_angles[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    VictorConfig init_config = victor_model.toVictorConfig(init_angles);
    victor_model.updateActual(init_config);
}

void SimWorld::makeTable()
{
    Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    Vector3f tc(1.7, 1.4, 0.9); //table corner
    Vector3f tld(.033, 0.033, tc.z); //table leg dims


    
    //table top
    gvl->insertBoxIntoMap(tc, tc + td,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0),
                          Vector3f(tc.x, tc.y, 0) + tld,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                          Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                          Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                          Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    

    Vector3f cavecorner;
    if(PEG_IN_HOLE)
    {
        cavecorner = Vector3f(1.7, 1.7, 0.9);
    }
    else
    {
        cavecorner = Vector3f(1.7, 2.0, 0.9);
    }
    Vector3f caveheight(0.0, 0.0, 0.4);
    Vector3f cavetopd(0.4, 0.5, 0.033);
    Vector3f cavesidedim(0.033, cavetopd.y, caveheight.z);
    Vector3f cavesideoffset(cavetopd.x, 0.0, 0.0);
    Vector3f caveholecorner = cavecorner + cavesideoffset + Vector3f(0, 0.15, 0.15);
    Vector3f caveholesize(.04, .15, .15);

    gvl->insertBoxIntoMap(cavecorner+caveheight,
                          cavecorner+caveheight+cavetopd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(cavecorner,
                          cavecorner+cavesidedim,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(cavecorner+cavesideoffset,
                          cavecorner+cavesideoffset+cavesidedim,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);


    if(PEG_IN_HOLE)
    {
        gvl->insertBoxIntoMap(caveholecorner,
                              caveholecorner + caveholesize,
                              TMP_MAP, PROB_OCCUPIED, 2);

        victor_model.m1_subtract_m2(SIM_OBSTACLES_MAP, TMP_MAP);
    }


    


    
    if(USE_KNOWN_OBSTACLES)
    {
        gvl->insertBoxIntoMap(tc, tc + td - Vector3f(0, .405, 0),
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        gvl->insertBoxIntoMap(cavecorner+caveheight,
                              cavecorner+caveheight+cavetopd,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        gvl->insertBoxIntoMap(cavecorner,
                              cavecorner+cavesidedim,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

        if(ALL_OBSTACLES_KNOWN)
        {
            gvl->insertBoxIntoMap(tc, tc + td,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

            gvl->insertBoxIntoMap(cavecorner+cavesideoffset,
                                  cavecorner+cavesideoffset+cavesidedim,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        }


        gvl->visualizeMap(SIM_OBSTACLES_MAP);
        gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
    }


}

void SimWorld::makeSlottedWall()
{

    std::cout << "Making slotted walls\n";
    // Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 0.0 * 0.0254); //table dimensions
    // Vector3f tc(1.7, 1.4, 0.9); //table corner
    // Vector3f tld(.033, 0.033, tc.z); //table leg dims



    double lower_wall_height = 1.1;
    double gap_height = .4;

    Vector3f lfwc(1.5, 1.6, 0.0); //lower front wall corner
    Vector3f lfwd(0.04, 1.5, lower_wall_height);
    
    Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height); //upper front call
    Vector3f ufwd(0.04, 1.5, 0.3);
    
    Vector3f mfwc(1.5, 1.8, 0);  //middle front wall
    Vector3f mfwd(0.04, 1.4, 1.5);
   
    Vector3f lswc = lfwc;  // lower side wall corner
    Vector3f lswd(1.5, 0.04, lower_wall_height); //lower side wall dims
    Vector3f cswc = ufwc; //close side wall corner
    Vector3f cswd(0.2, 0.04, 0.3);
    Vector3f fswc(1.95, 1.6, lower_wall_height); //far side wall corner
    Vector3f fswd(0.3, 0.04, 0.6);
    Vector3f mswc(1.95, 1.6, lower_wall_height+gap_height+.1); //far side wall corner
    Vector3f mswd(0.3, 0.04, 0.2);
    
    
    gvl->insertBoxIntoMap(lfwc, lfwc+lfwd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(ufwc, ufwc+ufwd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(mfwc, mfwc+mfwd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    
    gvl->insertBoxIntoMap(lswc, lswc+lswd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(cswc, cswc+cswd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(fswc, fswc+fswd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(mswc, mswc+mswd,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    
    if(USE_KNOWN_OBSTACLES)
    {
        gvl->insertBoxIntoMap(lfwc, lfwc+lfwd,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        gvl->insertBoxIntoMap(lswc, lswc+lswd,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

        gvl->insertBoxIntoMap(ufwc, ufwc+ufwd,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        gvl->insertBoxIntoMap(mfwc, mfwc+mfwd,
                              KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
        if(ALL_OBSTACLES_KNOWN)
        {
            gvl->insertBoxIntoMap(lswc, lswc+lswd,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
            gvl->insertBoxIntoMap(cswc, cswc+cswd,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
            gvl->insertBoxIntoMap(fswc, fswc+fswd,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
            gvl->insertBoxIntoMap(mswc, mswc+mswd,
                                  KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);


        }
        
    }
    gvl->visualizeMap(SIM_OBSTACLES_MAP);
    gvl->visualizeMap(KNOWN_OBSTACLES_MAP);

}
    
void SimWorld::initializeObstacles()
{

    if(MAKE_TABLE)
        makeTable();
    if(MAKE_SLOTTED_WALL)
        makeSlottedWall();
        
}

Maybe::Maybe<std::string> SimWorld::getCollisionLink(const VictorConfig &c)
{
    victor_model.resetQuery();
    victor_model.addQueryState(c);
    if(victor_model.countNumCollisions(SIM_OBSTACLES_MAP) > 0)
    {
        victor_model.resetQuery();
        for(auto &link_name: right_arm_collision_link_names)
        {
            victor_model.addQueryLink(c, link_name);
            if(victor_model.countNumCollisions(SIM_OBSTACLES_MAP) > 0)
            {
                std::cout << "Collides with " << link_name << "\n";
                return Maybe::Maybe<std::string>(link_name);
            }
        }
        std::cout << "Victor collides, but no links collide...\n";
        assert(false);
    }
    return Maybe::Maybe<std::string>();
}


/*
 *  Executes path until completion or collision.
 */
bool SimWorld::executePath(const Path &path, size_t &last_valid)
{
    if(VIDEO_VISUALIZE)
    {
        victor_model.resetQuery();
        
        gvl->visualizeMap(VICTOR_QUERY_MAP);
    }
    
    for(last_valid = 0; last_valid < path.size(); last_valid++)
    {
        std::vector<double> joint_angles = path[last_valid];
        VictorConfig c = victor_model.toVictorConfig(joint_angles.data());
        Maybe::Maybe<std::string> col_link = getCollisionLink(c);
        if(col_link.Valid())
        {
            std::cout << "Collision while executing!\n";
            std::vector<std::string> collision_links;
            collision_links.push_back(col_link.Get());

            /*
             * If collision with gripper add full gripper
             */
            if(std::find(victor_right_gripper_collision_names.begin(),
                         victor_right_gripper_collision_names.end(),
                         col_link.Get()) != victor_right_gripper_collision_names.end())
            {
                for(auto link: victor_right_gripper_collision_names)
                {
                    collision_links.push_back(link);
                }
            }

            std::vector<VictorConfig> cs;
            for(size_t j=last_valid; (j<last_valid + NUM_STEPS_FOR_ADDING_COLLISION) && j<path.size(); j++)
            {
                cs.push_back(victor_model.toVictorConfig(path[j].data()));
            }
            victor_model.addCollisionSet(cs, collision_links);
            last_valid--;
            return false;
        }
            
        victor_model.updateActual(c);
        usleep(EXECUTION_DELAY_us/2);
        victor_model.doVis();
        usleep(EXECUTION_DELAY_us/2);

    }
    // std::cout << "Path success\n";
    return true;
}



Path densifyPath(const Path &path, int densify_factor)
{
    // std::cout << "densifying path\n";
    Path dense_path;
    double dt = 1.0/(double)densify_factor;
    for(size_t i=0; i < (path.size()-1); i++)
    {
        for(int new_seg=0; new_seg < densify_factor; new_seg++)
        {
            const std::vector<double> &cur = path[i];
            const std::vector<double> &next = path[i+1];
            std::vector<double> interp;
            for(size_t j=0; j<cur.size(); j++)
            {
                interp.push_back(cur[j] + (next[j] - cur[j]) * new_seg *dt);
            }
            dense_path.push_back(interp);
        }
    
    }
    dense_path.push_back(path[path.size()-1]);
    return dense_path;
}



bool SimWorld::attemptPath(const Path &path)
{
    
    Path dense_path = densifyPath(path, 10);
    // std::cout << "Last state in given attempt " << path.back()[0] << "\n";
    // std::cout << "Last state in dense attempt " << dense_path.back()[0] << "\n";
    size_t last_valid;
    if(executePath(dense_path, last_valid))
    {
        return true;
    }
    

    std::cout << "backing up\n";
    std::cout << last_valid << "\n";
    Path backup;
    for(int i=0; i<30; i++)
    {
        backup.push_back(dense_path[last_valid]);
        if(last_valid == 0)
        {
            break;
        }
        last_valid--;
    }
    executePath(backup, last_valid);
    return false;
}













/***********************************
 **          REAL WORLD           **
 **********************************/
RealWorld::RealWorld()
{
    std::cout << "Creating real world\n";
    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->visualizeMap(KNOWN_OBSTACLES_MAP);

    ros::NodeHandle n;
    joint_sub = n.subscribe("/joint_states", 1, &RealWorld::jointStateCallback, this);
    attempt_path_client = n.serviceClient<gpu_voxel_planning::AttemptPathStart>("attempt_path_on_victor");
    get_attempt_status_client = n.serviceClient<gpu_voxel_planning::AttemptPathResult>("get_path_status");
    


    loadPointCloudFromFile();
    double init_angles[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    VictorConfig init_config = victor_model.toVictorConfig(init_angles);
    victor_model.updateActual(init_config);

    update_victor_from_messages = true;
    pos_updated = false;
}

RealWorld::~RealWorld()
{
    gvl.reset();
    victor_model.gvl.reset();
}

void RealWorld::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    VictorConfig cur;
    if( update_all_joint_count++ < 3)
    {
        for(size_t i=0; i<msg->position.size(); i++)
        {
            cur[msg->name[i]] = msg->position[i];
        }
        gvl->setRobotConfiguration(VICTOR_ROBOT_STATIONARY, cur);
        gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED);
    }
    else{

        std::vector<double> right_arm_angles(&msg->position[7], &msg->position[14]);
        cur = victor_model.toVictorConfig(right_arm_angles.data());
    }
    
    
    victor_model.updateActual(cur);
    victor_model.m1_subtract_m2(KNOWN_OBSTACLES_MAP, VICTOR_ACTUAL_MAP);
    victor_model.doVis();
    pos_updated = true;
}

void RealWorld::spinUntilUpdate()
{
    pos_updated = false;
    while(ros::ok() && !pos_updated)
    {
        ros::Duration(0.001).sleep();
        ros::spinOnce();
    }
    gvl->visualizeMap(VICTOR_ACTUAL_MAP);
}

void RealWorld::loadPointCloudFromFile()
{
    const std::pair<std::string, Eigen::Matrix3Xf> deserialized =
        LoadPointsetFromFile("./logs/point_cloud_latest.compressed");

    const Eigen::Matrix3Xf &mat = deserialized.second;
    std::cout << "Loading matrix of size " << mat.rows() << ", " << mat.cols() << "\n";
    std::vector<Vector3f> points;
    for(size_t i=0; i<mat.cols(); i++)
    {
        if(std::isnan(mat(0,i)) || std::isnan(mat(1,i)) || std::isnan(mat(2,i)))
        {
            continue;
        }
        // std::cout << mat(0, i) << ", " << mat(1,i) << ", " << mat(2,i) << "\n";
        Vector3f p(1.0+mat(0,i), 2.0+mat(1,i), mat(2,i));

        // if(p.x < 1.7 && p.y < 2.0 && p.z > 1.0)
        //     continue;
        
        points.push_back(p);
    }
    gvl->insertPointCloudIntoMap(points, KNOWN_OBSTACLES_MAP,  PROB_OCCUPIED);

}

bool RealWorld::attemptPath(const Path &path)
{
    if(VIDEO_VISUALIZE)
    {
        victor_model.resetQuery();
        
        gvl->visualizeMap(VICTOR_QUERY_MAP);
    }

    gpu_voxel_planning::AttemptPathStart srv;
    srv.request.path.points.resize(path.size());
    for(size_t i=0; i<path.size(); i++)
    {
        srv.request.path.points[i].positions = path[i];
    }
    
    if(attempt_path_client.call(srv))
    {
        std::cout << "New path message sent\n";
    }

    gpu_voxel_planning::AttemptPathResult path_res;
    bool path_finished = false;
    while(ros::ok() && !path_finished)
    {
        // std::cout << "path not yet finished\n";
        get_attempt_status_client.call(path_res);
        path_finished = path_res.response.finished;
        spinUntilUpdate();
        
    }
    std::cout << "Path finished\n";

    if(path_res.response.ci.collided)
    {
        gpu_voxel_planning::CollisionInformation &ci = path_res.response.ci;
        std::cout << "Collision found on path\n";

        std::vector<VictorConfig> col_configs;
        for(auto &traj_point: ci.collision_path.points)
        {
            col_configs.push_back(victor_model.toVictorConfig(traj_point.positions.data()));
        }
        std::vector<std::string> col_links = ci.collision_links;
        col_links.insert(col_links.end(), victor_right_gripper_collision_names.begin(),
                         victor_right_gripper_collision_names.end());
        victor_model.addCollisionSet(col_configs, col_links);
        std::cout << "added " << col_configs.size() << " collision configs\n";
        std::cout << "for links \n";
        for(auto &link_name: col_links)
        {
            std::cout << link_name << "\n";
        }
        spinUntilUpdate();

        return false;
    }
    

    return true;
}
