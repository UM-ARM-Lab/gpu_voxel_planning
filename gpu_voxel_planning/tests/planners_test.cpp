#include "gpu_voxels_victor.hpp"
#include <gtest/gtest.h>
#include <iostream>

#include "victor_planning.hpp"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


#define PROB_OCCUPIED eBVM_OCCUPIED
#define COLLISION_SET "chs_0"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace gpu_voxels_planner;



TEST(Planners, RRTConnectPlanner)
{
    GpuVoxelsVictor victor_model;
    
    double start[] = {-0.15,0,0,0,0,0,0};
    double goal[] = {-0.15, 1.2, 0, -0.5, 0, 1.0, 0};
    robot::JointValueMap sconfig = victor_model.toVictorConfig(start);
    robot::JointValueMap gconfig = victor_model.toVictorConfig(goal);
    victor_model.updateActual(sconfig);


    // Inflate the sampled world because corners could be cut slightly by rrt
    victor_model.gvl->insertBoxIntoMap(Vector3f(1.6, 1.4, 0.5), Vector3f(3.2 ,1.7,1.7), 
                                       SAMPLED_WORLD_MAP, PROB_OCCUPIED, 2);
    
    victor_model.gvl->insertBoxIntoMap(Vector3f(1.7, 1.5, 0.6), Vector3f(3.0 ,1.5,1.5), 
                                       SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    victor_model.gvl->visualizeMap(SIM_OBSTACLES_MAP);


    VictorRRTConnect planner(&victor_model);

    Maybe::Maybe<Path> path = planner.planPathConfig(sconfig, gconfig);

    ASSERT_TRUE(path.Valid()) << "Invalid path. Code not necessarily broken, but test cannot continue";

    for(auto p: path.Get())
    {
        victor_model.addQueryState(victor_model.toVictorConfig(p.data()));
    }
    EXPECT_EQ(0, victor_model.countIntersect(VICTOR_QUERY_MAP, SIM_OBSTACLES_MAP));
    EXPECT_FALSE(victor_model.overlaps(VICTOR_QUERY_MAP, SIM_OBSTACLES_MAP));

    // victor_model.visPath(path.Get());
    // std::cout << "Waiting for user input to exit\n";
    // std::string dummy;
    // std::getline(std::cin, dummy);


}


GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

