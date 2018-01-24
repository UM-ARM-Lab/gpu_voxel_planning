#include "victor_validator.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <thrust/extrema.h>

#include <thread>
#include <chrono>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels;
using namespace vvhelpers;
namespace bfs = boost::filesystem;


#define PROB_OCCUPIED BitVoxelMeaning(255)


std::vector<std::string> right_arm_joint_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2",
        "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5",
        "victor_right_arm_joint_6", "victor_right_arm_joint_7"};

std::vector<std::string> right_arm_collision_link_names{"victor_right_arm_link_5",
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


VictorValidator::VictorValidator(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{

    si_ = si;
    stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(200, 200, 200, 0.02);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP,"victor_tmp"); //map for queries on victor validity
    gvl->addMap(MT_PROBAB_VOXELMAP,"victor"); //map for victors current state
    gvl->addMap(MT_PROBAB_VOXELMAP,"env");
    // gvl->addMap(MT_BITVECTOR_VOXELMAP, "env");
    gvl->addMap(MT_BITVECTOR_VOXELLIST,"solutions");
    gvl->addMap(MT_PROBAB_VOXELMAP,"victor_swept_volume");
    gvl->addMap(MT_PROBAB_VOXELMAP,"query");
    gvl->addMap(MT_DISTANCE_VOXELMAP, "distancemap");
    std::cout << "Adding robot\n";

    gvl->addRobot("victor_robot", "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor.urdf", false);  

    gvl->visualizeMap("env");

    PERF_MON_ENABLE("pose_check");
    PERF_MON_ENABLE("motion_check");
    PERF_MON_ENABLE("motion_check_lv");
}




VictorValidator::~VictorValidator()
{
    std::cout << "Reset called\n";
    gvl.reset(); // Not even required, as we use smart pointers.
}

void VictorValidator::moveObstacle()
{
    gvl->clearMap("env");

    // use this to animate a single moving box obstacle
    //gvl->insertBoxIntoMap(Vector3f(2.0, x ,0.0), Vector3f(2.2, x + 0.2 ,1.2), "env", eBVM_OCCUPIED, 2);

    gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2), "env", PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(1.8,1.8,0.8), Vector3f(2.0,2.0,1.2), "env", PROB_OCCUPIED, 2);
    // gvl->insertBoxIntoMap(Vector3f(1.8,1.8,0.0), Vector3f(2.0,2.0,1.2), "env", eBVM_OCCUPIED, 2);
    // gvl->insertBoxIntoMap(Vector3f(1.1,1.1,1.2), Vector3f(1.9,1.9,1.3), "env", eBVM_OCCUPIED, 2);
    // gvl->insertBoxIntoMap(Vector3f(0.0,0.0,0.0), Vector3f(3.0,3.0,0.01), "env", eBVM_OCCUPIED, 2);
    gvl->visualizeMap("env");
}

void VictorValidator::setVictorPosition(robot::JointValueMap joint_positions)
{
    gvl->clearMap("victor");
    gvl->setRobotConfiguration("victor_robot", joint_positions);
    // gvl->insertRobotIntoMap("victor_robot", "victor", eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("victor_robot", "victor", PROB_OCCUPIED);
    gvl->insertRobotIntoMap("victor_robot", "victor_swept_volume", PROB_OCCUPIED);

    gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap("env");
    voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
  
    obstacles->subtract(gvl->getMap("victor_swept_volume")->as<voxelmap::ProbVoxelMap>());

    
}



struct CompareProbDist
{
    typedef thrust::tuple<ProbabilisticVoxel, uint32_t> ProbDist;
    Vector3ui dims;
    CompareProbDist(const Vector3ui &map_dims)
        {
            dims = map_dims;
        }
    

    /*
     *Returns true if lhs < rhs
     * i.e lhs is occupied
     * if rhs is occupied, the distance value is lower
     *
     */
    __host__ __device__
    bool operator()(ProbDist lhs, ProbDist rhs)
        {
            return true;
            // Vector3i pos = linearIndexToCoordinates(thrust::get<2>(lhs), dims);
            
                
            // return (thrust::get<0>(lhs).getOccupancy() == MAX_PROBABILITY) &&
            //     ((thrust::get<0>(rhs).getOccupancy() != MAX_PROBABILITY) ||
            //      (thrust::get<1>(lhs).squaredObstacleDistance(pos) <
            //       thrust::get<1>(rhs).squaredObstacleDistance(pos)));
        }

};

void VictorValidator::determineVictorDist()
{
    boost::shared_ptr<voxelmap::ProbVoxelMap> victor = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap("victor"));

    boost::shared_ptr<voxelmap::DistanceVoxelMap> dist_map = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap("distancemap"));

    thrust::counting_iterator<uint32_t> count(0);
    uint32_t s = dist_map->getVoxelMapSize();
    thrust::min_element(
        thrust::device_system_tag(),
        thrust::make_zip_iterator(thrust::make_tuple(victor->getDeviceDataPtr(),
                                                     count)),
        thrust::make_zip_iterator(thrust::make_tuple(victor->getDeviceDataPtr() + s,
                                                     count + s)),
        CompareProbDist(dist_map->getDimensions())
        );

    // Vector3i pos = linearIndexToCoordinates(thrust::get<2>(*min_probdist)
    // std::cout << "min dist is " << thrust::get<1>
}




void VictorValidator::addCollisionPoints(CollisionInformation collision_info)
{
    if(collision_info.collision)
    {
        robot::JointValueMap cur_joints, extended_joints;
        for(size_t i = 0; i < collision_info.joints.size(); i++)
        {
            cur_joints[right_arm_joint_names[i]] = collision_info.joints[i];
            extended_joints[right_arm_joint_names[i]] = collision_info.joints[i] + 0.05 * collision_info.dirs[i];
            // std::cout << 0.05 * c.dirs[i] << ", ";
            // std::cout << c.joints[i] << ", ";
        }
        // std::cout << "\n";
        
        gvl->setRobotConfiguration("victor_robot", extended_joints);
        
        // gvl->insertRobotIntoMap("victor_robot", "env", PROB_OCCUPIED);

        RobotInterfaceSharedPtr rob = gvl->getRobot("victor_robot");
        const MetaPointCloud* clouds = rob->getTransformedClouds();
        // clouds->syncToHost();
        rob->syncToHost();


        for(auto collision_link_name: right_arm_collision_link_names)
        {
            // std::cout << collision_link_name << "\n";
            int16_t cloud_num = clouds->getCloudNumber(collision_link_name);
            uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
            const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
            const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
            gvl->insertPointCloudIntoMap(cloud, "env", PROB_OCCUPIED);
        }
            

        gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap("env");
        // boost::shared_ptr<voxelmap::ProbVoxelMap> obstacles(obstacles_ptr->as<voxelmap::ProbVoxelMap>());
        // voxelmap::ProbVoxelMap* obstacles(obstacles_ptr->as<voxelmap::ProbVoxelMap>());
        boost::shared_ptr<voxelmap::ProbVoxelMap> obstacles = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(obstacles_ptr);
  
        obstacles->subtract(gvl->getMap("victor_swept_volume")->as<voxelmap::ProbVoxelMap>());


        // boost::shared_ptr<voxelmap::DistanceVoxelMap> dist_map(gvl->getMap("distancemap")->as<voxelmap::DistanceVoxelMap>());
        
        boost::shared_ptr<voxelmap::DistanceVoxelMap> dist_map = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap("distancemap"));



        std::cout << "Obstacle dist: " << dist_map->getObstacleDistance(Vector3ui(10,100,100)) << "\n";

        // auto start = std::chrono::steady_clock::now();
        dist_map->clearMap();
        dist_map->mergeOccupied(obstacles);
        dist_map->jumpFlood3D(cMAX_THREADS_PER_BLOCK, 0, false);
        // auto end = std::chrono::steady_clock::now();

        // std::cout << "Elapsed time: "
        //           << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "us \n";

        // for(int i=0; i<200; i++)
        // {
        //     for(int j=0; j<200; j++)
        //     {
        //         for(int k=0; k<200; k++)
        //         {
        //             float dist = dist_map->getObstacleDistance(Vector3ui(i,j,k));
        //             if(dist < 10000)
        //             {
        //                 std::cout << "(" <<i << ", " << j << ", " << k << "): " << dist<<"\n";
        //             }
        //         }
        //     }
        // }

                
    }
}



void VictorValidator::doVis()
{
    // tell the visualier that the map has changed:

    // gvl->visualizeMap("victor_tmp");
    gvl->visualizeMap("victor");
    gvl->visualizeMap("env");
    gvl->visualizeMap("solutions");
    // gvl->visualizeMap("query");
}

void VictorValidator::visualizeSolution(ob::PathPtr path)
{
    gvl->clearMap("solutions");

    PERF_MON_SUMMARY_PREFIX_INFO("pose_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check_lv");

    std::cout << "Robot consists of " << gvl->getRobot("victor_robot")->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;

    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();


    for(size_t step = 0; step < solution->getStateCount(); ++step)
    {

        const double *values = solution->getState(step)->as<ob::RealVectorStateSpace::StateType>()->values;
        
        robot::JointValueMap state_joint_values = toRightJointValueMap<const double*>(values);

        // update the robot joints:
        gvl->setRobotConfiguration("victor_robot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("victor_robot", "solutions", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
    }

    // gvl->visualizeMap("solutions");

}

void VictorValidator::insertStartAndGoal(const ob::ScopedState<> &start, const ob::ScopedState<> &goal) const
{

    gvl->clearMap("query");
    robot::JointValueMap state_joint_values = toRightJointValueMap<ob::ScopedState<>>(start);

    // update the robot joints:
    gvl->setRobotConfiguration("victor_robot", state_joint_values);
    gvl->insertRobotIntoMap("victor_robot", "query", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

    state_joint_values = toRightJointValueMap<ob::ScopedState<>>(goal);

    // update the robot joints:
    gvl->setRobotConfiguration("victor_robot", state_joint_values);
    gvl->insertRobotIntoMap("victor_robot", "query", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));

}

bool VictorValidator::isValid(const ob::State *state) const
{

    PERF_MON_START("inserting");

    std::lock_guard<std::mutex> lock(g_i_mutex);

    gvl->clearMap("victor_tmp");

    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    
    robot::JointValueMap state_joint_values = toRightJointValueMap<const double*>(values);


    // update the robot joints:
    gvl->setRobotConfiguration("victor_robot", state_joint_values);
    // insert the robot into the map:
    gvl->insertRobotIntoMap("victor_robot", "victor_tmp", PROB_OCCUPIED);

    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Pose Insertion", "pose_check");

    PERF_MON_START("coll_test");
    size_t num_colls_pc = gvl->getMap("victor_tmp")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("env")->as<voxelmap::ProbVoxelMap>());
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "pose_check");

    //std::cout << "Validity check on state ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc << " colls." << std::endl;

    return num_colls_pc == 0;
}

bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                       std::pair< ob::State*, double > & lastValid) const
{

    //    std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //    std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //    std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;


    std::lock_guard<std::mutex> lock(g_j_mutex);
    gvl->clearMap("victor_tmp");

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    //std::cout << "Called interpolating motion_check_lv to evaluate " << nd << " segments" << std::endl;

    PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))

            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;

            }
        }

        si_->freeState(test);

    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;
}







//bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2,
//                                  std::pair< ob::State*, double > & lastValid) const
//{

//    std::lock_guard<std::mutex> lock(g_j_mutex);
//    gvl->clearMap("victor_tmp");

//    /* assume motion starts in a valid configuration so s1 is valid */

//    bool result = true;
//    int nd = stateSpace_->validSegmentCount(s1, s2);

//    static size_t bar(0);
//    bar++;


//    if(nd > 249)
//    {
//        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
//        exit(1);
//    }

//    if (nd > 1)
//    {
//        PERF_MON_START("inserting");
//        /* temporary storage for the checked state */
//        ob::State *test = si_->allocState();

//        for (int j = 1; j < nd; ++j)
//        {
//            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


//            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

//            robot::JointValueMap state_joint_values;
//            state_joint_values["shoulder_pan_joint"] = values[0];
//            state_joint_values["shoulder_lift_joint"] = values[1];
//            state_joint_values["elbow_joint"] = values[2];
//            state_joint_values["wrist_1_joint"] = values[3];
//            state_joint_values["wrist_2_joint"] = values[4];
//            state_joint_values["wrist_3_joint"] = values[5];

//            // update the robot joints:
//            gvl->setRobotConfiguration("victor_tmp", state_joint_values);
//            // insert the robot into the map:
//            gvl->insertRobotIntoMap("victor_robot", "victor_tmp", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + j));

//        }
//        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
//        //PERF_MON_ADD_DATA_NONTIME_P("Num Voxels in Robot Voxellist after insertion of motion", float(gvl->getMap("victor_tmp")->getDimensions().x), "motion_check_lv");

//        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check_lv");


//        //gvl->visualizeMap("victor_tmp");
//        PERF_MON_START("coll_test");

//        BitVectorVoxel bits_in_collision;

//        size_t num_colls;

//        num_colls = gvl->getMap("victor_tmp")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("env")->as<voxelmap::ProbVoxelMap>(), bits_in_collision);

////        std::cout << "Detected " << num_colls << " collisions " << std::endl;
////        std::cout << "with bits \n" << bits_in_collision << std::endl;

//        for(size_t seg = eBVM_SWEPT_VOLUME_START + 1;  seg <= eBVM_SWEPT_VOLUME_END; seg++)

//        {
//            if(bits_in_collision.bitVector().getBit(seg))
//            {
//              double frac = (double)(seg - eBVM_SWEPT_VOLUME_START -1) / (double)nd;
//              stateSpace_->interpolate(s1, s2, frac, test);
//              if (lastValid.first != nullptr)
//              {
//                lastValid.first = test;
//              }
//              lastValid.second = frac;
//              break;
//            }

//        }

//        result = (num_colls == 0);


//        if (result)
//        {
//            if (!si_->isValid(s2))
//            {
//                lastValid.second = (double)(nd - 1) / (double)nd;
//                if (lastValid.first != nullptr)
//                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
//                result = false;
//            }
//        }




//        si_->freeState(test);

//        //std::cout << "CheckMotion1 # " << bar << " for " << nd << " segments. Resulting in " << num_colls << " colls." << std::endl;
//        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check_lv");


//    }



//    if (result)
//        valid_++;
//    else
//        invalid_++;



//    return result;

//}









bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    gvl->clearMap("victor_tmp");


    //        std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //        std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //        std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;



    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        PERF_MON_START("inserting");

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            
            robot::JointValueMap state_joint_values = toRightJointValueMap<const double*>(values);

            // update the robot joints:
            gvl->setRobotConfiguration("victor_robot", state_joint_values);
            // insert the robot into the map:
            gvl->insertRobotIntoMap("victor_robot", "victor_tmp", PROB_OCCUPIED);

        }
        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check");

        si_->freeState(test);

        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check");

        //gvl->visualizeMap("victor_tmp");
        PERF_MON_START("coll_test");
        size_t num_colls_pc = gvl->getMap("victor_tmp")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("env")->as<voxelmap::ProbVoxelMap>());
        //std::cout << "CheckMotion1 for " << nd << " segments. Resulting in " << num_colls_pc << " colls." << std::endl;
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check");

        result = (num_colls_pc == 0);

    }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;


}


template<typename T>
robot::JointValueMap vvhelpers::toRightJointValueMap(const T values)
{
    robot::JointValueMap jvm;
    for(size_t i=0; i<right_arm_joint_names.size(); i++)
    {
        jvm[right_arm_joint_names[i]] = values[i];
    }
    return jvm;
}
