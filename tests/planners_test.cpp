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

TEST(GpuVoxelVictor, collisions)
{


    GpuVoxelsVictor victor_model;
    
    double start[] = {-0.15,0,0,0,0,0,0};
    double goal[] = {-0.15, 1.2, 0, -0.5, 0, 1.0, 0};

    
    robot::JointValueMap sconfig = victor_model.toVictorConfig(start);
    robot::JointValueMap gconfig = victor_model.toVictorConfig(goal);
    victor_model.updateActual(sconfig);
    


    // victor_model.gvl->insertBoxIntoMap(Vector3f(1.6, 1.4, 0.5), Vector3f(3.0 ,1.5,1.5), 
    //                                    COLLISION_SET, PROB_OCCUPIED, 2);
    // victor_model.num_observed_sets = 1;


    ASSERT_TRUE(victor_model.queryFreeConfiguration(sconfig)) << "Victor at initial position is in collision";
    ASSERT_TRUE(victor_model.queryFreeConfiguration(gconfig)) << "Victor at goal position is in collision";

    VictorThresholdRRTConnect planner(&victor_model, true, false);

    Maybe::Maybe<Path> path = planner.planPathConfig(sconfig, gconfig);

    ASSERT_TRUE(path.Valid()) << "Invalid path. Code not necessarily broken, but test cannot continue";

    victor_model.visPath(path.Get());

    for(auto values: path.Get())
    {
        VictorConfig on_path_config = victor_model.toVictorConfig(values.data());
        EXPECT_TRUE(victor_model.queryFreeConfiguration(on_path_config)) << "Collision found on path";
    }

    


    std::cout << "Waiting for user input to exit\n";
    std::string dummy;
    std::getline(std::cin, dummy);

}



GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

