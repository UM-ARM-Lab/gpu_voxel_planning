#include "gpu_voxels_victor.hpp"
#include <gtest/gtest.h>
#include <iostream>


#define PROB_OCCUPIED eBVM_OCCUPIED
#define ENV_MAP "env_map"

TEST(GpuVoxelVictor, collisions)
{

    GpuVoxelsVictor victor_model;
    double angles[] = {0,0,0,0,0,0,0};
    robot::JointValueMap map = victor_model.toVictorConfig(angles);
    bool is_valid = victor_model.queryFreeConfiguration(map);
    size_t col_count = victor_model.countNumCollisions(map);
    EXPECT_EQ(0, col_count) << "Victor at initial position has collisions";
    EXPECT_TRUE(is_valid) << "Victor at initial position is in collision";


    victor_model.gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                                       ENV_MAP, PROB_OCCUPIED, 2);


    is_valid = victor_model.queryFreeConfiguration(map);
    col_count = victor_model.countNumCollisions(map);
    EXPECT_TRUE(col_count > 0) << "Victor with box obstacle has no collision";
    EXPECT_TRUE(!is_valid) << "Victor with box obstacle is not in collision";

    
    // victor_model.updateVictorPosition(map);
    // victor_model.doVis();
    // int dummy;
    // std::cin >> dummy;
}



GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

