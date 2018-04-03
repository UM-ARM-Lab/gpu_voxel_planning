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
#define VICTOR_ROBOT "victor_robot"




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



GpuVoxelsVictor::GpuVoxelsVictor()
{
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


}


void GpuVoxelsVictor::updateVictorPosition(robot::JointValueMap joint_positions)
{
    std::lock_guard<boost::recursive_timed_mutex> g(gvl->getMap(VICTOR_ACTUAL_MAP)->m_mutex);
    gvl->clearMap(VICTOR_ACTUAL_MAP);
    gvl->setRobotConfiguration(VICTOR_ROBOT, joint_positions);
    // gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, eBVM_OCCUPIED);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_SWEPT_VOLUME_MAP, PROB_OCCUPIED);

    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(ENV_MAP);
    voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
  
    obstacles->subtract(gvl->getMap(VICTOR_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());
}


size_t GpuVoxelsVictor::countNumCollisions(const robot::JointValueMap &joint_values_map)
{
    PROFILE_START(ISVALID_INSERTION);
    PROFILE_START(QUERY_INSERTION);
    
    gvl->clearMap(VICTOR_QUERY_MAP);

    // update the robot joints:
    gvl->setRobotConfiguration(VICTOR_ROBOT, joint_values_map);
    // insert the robot into the map:
    gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_QUERY_MAP, PROB_OCCUPIED);

    PROFILE_RECORD(ISVALID_INSERTION);
    PROFILE_RECORD(QUERY_INSERTION);


    PROFILE_START(ISVALID_COLLISION_TEST);

    size_t num_colls_pc = gvl->getMap(VICTOR_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(ENV_MAP)->as<voxelmap::ProbVoxelMap>());


    PROFILE_RECORD(ISVALID_COLLISION_TEST);
    return num_colls_pc;

}


bool GpuVoxelsVictor::queryFreeConfiguration(const robot::JointValueMap &joint_values_map)
{
    return countNumCollisions(joint_values_map) == 0;
}








void GpuVoxelsVictor::addCollisionPoints(CollisionInformation collision_info)
{
    std::lock_guard<boost::recursive_timed_mutex> g(gvl->getMap(VICTOR_ACTUAL_MAP)->m_mutex);
    if(!collision_info.collision)
    {
        std::cout << "Asked to add collision, but provided CollisionInformation indicates no collision\n";
        return;
    }
    robot::JointValueMap cur_joints, extended_joints;
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

    
    gvl->setRobotConfiguration(VICTOR_ROBOT, extended_joints);
        
    // gvl->insertRobotIntoMap(VICTOR_ROBOT, ENV_MAP, PROB_OCCUPIED);

    RobotInterfaceSharedPtr rob = gvl->getRobot(VICTOR_ROBOT);
    const MetaPointCloud* clouds = rob->getTransformedClouds();
    // clouds->syncToHost();
    rob->syncToHost();


    // Add only the links in collision
    // for(auto collision_link_name: right_arm_collision_link_names)
    std::cout << "Collided with ";
    for(auto collision_link_name: collision_info.links_in_contact)
    {
        std::cout << collision_link_name << ", ";
        // std::cout << collision_link_name << "\n";
        int16_t cloud_num = clouds->getCloudNumber(collision_link_name);
        uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
        const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
        const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
        gvl->insertPointCloudIntoMap(cloud, ENV_MAP, PROB_OCCUPIED);
    }
    std::cout << "\n";
            

    //Add voxels to map
    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(ENV_MAP);
    // boost::shared_ptr<voxelmap::ProbVoxelMap> obstacles(obstacles_ptr->as<voxelmap::ProbVoxelMap>());
    // voxelmap::ProbVoxelMap* obstacles(obstacles_ptr->as<voxelmap::ProbVoxelMap>());
    boost::shared_ptr<voxelmap::ProbVoxelMap> obstacles = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(obstacles_ptr);
  
    obstacles->subtract(gvl->getMap(VICTOR_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());

        
    boost::shared_ptr<voxelmap::DistanceVoxelMap> dist_map = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap(OBSTACLE_DISTANCE_MAP));



    // std::cout << "Obstacle dist: " << dist_map->getObstacleDistance(Vector3ui(10,100,100)) << "\n";

    // auto start = std::chrono::steady_clock::now();
    dist_map->clearMap();
    dist_map->mergeOccupied(obstacles);
    dist_map->jumpFlood3D(cMAX_THREADS_PER_BLOCK, 0, false);
    // determineVictorDist();
    // auto end = std::chrono::steady_clock::now();

    // std::cout << "Elapsed time: "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "us \n";

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
    // tell the visualier that the map has changed:

    // gvl->visualizeMap(VICTOR_QUERY_MAP);

    // BitVoxelMeaning col = eBVM_COLLISION;
    // boost::shared_ptr<voxelmap::DistanceVoxelMap> env_map_red = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap(ENV_MAP_RED));

    // env_map_red->mergeOccupied(gvl->getMap(ENV_MAP), Vector3f(), &col);

    
    arc_helpers::DoNotOptimize<bool>(gvl->visualizeMap(VICTOR_ACTUAL_MAP, true));
    gvl->visualizeMap(ENV_MAP, true);
    // gvl->visualizeMap(ENV_MAP_RED);

    // gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP);
    // gvl->visualizeMap(VICTOR_PATH_ENDPOINTS_MAP);
    // usleep(100000);
    // std::cout << "Doing Vis\n";
}



// void GpuVoxelsVictor::visualizeSolution(ob::PathPtr path)
void GpuVoxelsVictor::visualizeSolution(const std::vector<robot::JointValueMap> &joint_maps)
{
    gvl->clearMap(VICTOR_PATH_SOLUTION_MAP);
    
    // std::cout << "Robot consists of " << gvl->getRobot(VICTOR_ROBOT)->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;
    for(size_t step = 0; step < joint_maps.size(); step++)
    {
        PROFILE_START(INSERT_VIZ_SOLUTION);
        auto &state_joint_values = joint_maps[step];
        // update the robot joints:
        gvl->setRobotConfiguration(VICTOR_ROBOT, state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_SOLUTION_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
        PROFILE_RECORD(INSERT_VIZ_SOLUTION);
    }

    std::cout << "visualizing solution\n";
    gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP, true);
}


void GpuVoxelsVictor::hideSolution()
{
    visualizeSolution(std::vector<robot::JointValueMap>());
    // std::cout << "Hiding solution\n";
    // gvl->clearMap(VICTOR_PATH_SOLUTION_MAP);
    // arc_helpers::DoNotOptimize<bool>(gvl->visualizeMap(VICTOR_PATH_SOLUTION_MAP, true));
    // usleep(300000);
}



// void VictorValidator::insertStartAndGoal(const ob::ScopedState<> &start, const ob::ScopedState<> &goal) const
// {

//     gvl->clearMap(VICTOR_PATH_ENDPOINTS_MAP);
//     robot::JointValueMap state_joint_values = toRightJointValueMap<ob::ScopedState<>>(start);

//     // update the robot joints:
//     gvl->setRobotConfiguration(VICTOR_ROBOT, state_joint_values);
//     gvl->insertRobotIntoMap(VICTOR_ROBOT, VICTOR_PATH_ENDPOINTS_MAP, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

//     state_joint_values = toRightJointValueMap<ob::ScopedState<>>(goal);

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



robot::JointValueMap GpuVoxelsVictor::toRightJointValueMap(const double* values)
{
    robot::JointValueMap jvm;
    for(size_t i=0; i<right_arm_joint_names.size(); i++)
    {
        jvm[right_arm_joint_names[i]] = values[i];
    }
    return jvm;
}


