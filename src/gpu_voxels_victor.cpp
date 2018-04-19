#include "gpu_voxels_victor.hpp"


#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <arc_utilities/arc_helpers.hpp>


#include <boost/thread/lock_guard.hpp>
#include <boost/thread/locks.hpp>
#include <mutex>

#include "common_names.hpp"

// #define PROB_OCCUPIED BitVoxelMeaning(255)
#define PROB_OCCUPIED eBVM_OCCUPIED

#define VICTOR_ACTUAL_MAP "victor_actual_map"
#define VICTOR_QUERY_MAP "victor_query_map"
#define ENV_MAP "env_map"

#define VICTOR_SWEPT_VOLUME_MAP "victor_swept_volume_map"
#define VICTOR_PATH_ENDPOINTS_MAP "victor_path_endpoints_map"
#define VICTOR_PATH_SOLUTION_MAP "victor_path_solutions_map"
#define OBSTACLE_DISTANCE_MAP "obstacle_distance_map"

#define SIM_OBSTACLES_MAP "sim_obstacles_map"
#define VICTOR_ROBOT "victor_robot"
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
    // gvl->addMap(MT_BITVECTOR_VOXELMAP, ENV_MAP);
    gvl->addMap(MT_BITVECTOR_VOXELLIST,VICTOR_PATH_SOLUTION_MAP);
    // gvl->addMap(MT_BITVECTOR_VOXELMAP, ENV_MAP_RED);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_SWEPT_VOLUME_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_PATH_ENDPOINTS_MAP);
    gvl->addMap(MT_DISTANCE_VOXELMAP, OBSTACLE_DISTANCE_MAP);
    
    gvl->addRobot(VICTOR_ROBOT, "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor.urdf", false);
    
    SEEN_OBSTACLE_SETS.resize(NUM_SETS);
    for(int i=0; i < NUM_SETS; i++)
    {
        SEEN_OBSTACLE_SETS[i] = "seen_obstacles_" + std::to_string(i);
        gvl->addMap(MT_PROBAB_VOXELMAP, SEEN_OBSTACLE_SETS[i]);
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[i]);
    }
    gvl->visualizeMap(VICTOR_ACTUAL_MAP);


}

void GpuVoxelsVictor::insertVictorIntoMap(const VictorConfig &c, const std::string &map_name)
{
    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, map_name, PROB_OCCUPIED);
}


void GpuVoxelsVictor::updateActual(const VictorConfig &c)
{
    gvl->clearMap(VICTOR_ACTUAL_MAP);
    gvl->setRobotConfiguration(VICTOR_ROBOT, c);
    // gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, eBVM_OCCUPIED);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_SWEPT_VOLUME_MAP, PROB_OCCUPIED);

    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(ENV_MAP);
    voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
  
    obstacles->subtract(gvl->getMap(VICTOR_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());
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
size_t GpuVoxelsVictor::countNumCollisions()
{
    PROFILE_START(ISVALID_COLLISION_TEST);

    size_t num_colls_pc = gvl->getMap(VICTOR_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(ENV_MAP)->as<voxelmap::ProbVoxelMap>());


    PROFILE_RECORD(ISVALID_COLLISION_TEST);
    return num_colls_pc;

}


size_t GpuVoxelsVictor::countNumCollisions(const VictorConfig &c)
{
    PROFILE_START(ISVALID_INSERTION);
    PROFILE_START(QUERY_INSERTION);
    
    resetQuery();

    addQueryState(c);

    PROFILE_RECORD(ISVALID_INSERTION);
    PROFILE_RECORD(QUERY_INSERTION);
    return countNumCollisions();
}


bool GpuVoxelsVictor::queryFreeConfiguration(const VictorConfig &c)
{
    return countNumCollisions(c) == 0;
}








void GpuVoxelsVictor::addCollisionPoints(CollisionInformation collision_info)
{
    std::lock_guard<boost::recursive_timed_mutex> g(gvl->getMap(VICTOR_ACTUAL_MAP)->m_mutex);
    if(!collision_info.collision)
    {
        std::cout << "Asked to add collision, but provided CollisionInformation indicates no collision\n";
        return;
    }
    VictorConfig cur_joints, extended_joints;
    for(size_t i = 0; i < collision_info.joints.size(); i++)
    {
        cur_joints[right_arm_joint_names[i]] = collision_info.joints[i];
        extended_joints[right_arm_joint_names[i]] = collision_info.joints[i] + 0.05 * collision_info.dirs[i];
    }

    gvl->setRobotConfiguration(VICTOR_ROBOT, cur_joints);

    gvl->clearMap(VICTOR_ACTUAL_MAP);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    
    int dist = determineVictorDist();
    // if(dist <= 1)
    // {
    //     std::cout << "Already knew victor would be in collision. Skipping\n";
    //     return;
    // }
            

    // Update robot to be slightly into collision object.
    // Insert only the last few links (heuristic to avoid adding too many points)


    addCollisionLinks(extended_joints, collision_info.links_in_contact, ENV_MAP);
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
        const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
        const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
        gvl->insertPointCloudIntoMap(cloud, map_name, PROB_OCCUPIED);
    }

    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(map_name);
    boost::shared_ptr<voxelmap::ProbVoxelMap> obstacles = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(obstacles_ptr);
  
    obstacles->subtract(gvl->getMap(VICTOR_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());

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
}



// void GpuVoxelsVictor::visualizeSolution(ob::PathPtr path)
void GpuVoxelsVictor::visualizeSolution(const std::vector<VictorConfig> &configs)
{
    gvl->clearMap(VICTOR_PATH_SOLUTION_MAP);

    // Some new voxels seem to be needed to force the map to refresh in the visualizer
    gvl->insertBoxIntoMap(Vector3f(0.0,0.0,0.0), Vector3f(0.02,0.02,0.02),
                          VICTOR_PATH_SOLUTION_MAP, PROB_OCCUPIED, 2);
        
    
    // std::cout << "Robot consists of " << gvl->getRobot(VICTOR_ROBOT)->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;
    for(size_t step = 0; step < configs.size(); step++)
    {
        PROFILE_START(INSERT_VIZ_SOLUTION);
        auto &state_joint_values = configs[step];
        // update the robot joints:
        gvl->setRobotConfiguration(VICTOR_ROBOT, state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_SOLUTION_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
        PROFILE_RECORD(INSERT_VIZ_SOLUTION);
    }

    gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP, true);
}


void GpuVoxelsVictor::hideSolution()
{
    visualizeSolution(std::vector<VictorConfig>());
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






/*************************************
 **       SIM OBSTACLES             **
 ************************************/
SimWorld::SimWorld()
{
    std::cout << "Creating sim world\n";
    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->addMap(MT_PROBAB_VOXELMAP, SIM_OBSTACLES_MAP);
    gvl->visualizeMap(SIM_OBSTACLES_MAP);

    double init_angles[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    VictorConfig init_config = victor_model.toVictorConfig(init_angles);
    victor_model.updateActual(init_config);
}

void SimWorld::initializeObstacles()
{
    Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    Vector3f tc(1.5, 1.6, 0.9); //table corner
    Vector3f tld(.033, 0.033, tc.z); //table leg dims
    
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
    

    
    gvl->visualizeMap(SIM_OBSTACLES_MAP);

}


bool SimWorld::executePath(const Path &path)
{
    for(auto joint_angles: path)
    {
        VictorConfig c = victor_model.toVictorConfig(joint_angles.data());
        victor_model.updateActual(c);
        victor_model.doVis();
        usleep(50000);
    }
    std::cout << "Path success!\n";
    return true;
}
