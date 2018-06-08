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
    size_t col_count = victor_model.countTotalNumCollisionsForConfig(map);
    EXPECT_EQ(0, col_count) << "Victor at initial position has collisions";
    EXPECT_TRUE(is_valid) << "Victor at initial position is in collision";


    victor_model.gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                                       ENV_MAP, PROB_OCCUPIED, 2);


    // is_valid = victor_model.queryFreeConfiguration(map);
    // col_count = victor_model.countTotalNumCollisionsForConfig(map);
    victor_model.resetQuery();
    victor_model.addQueryState(map);
    col_count = victor_model.countNumCollisions(ENV_MAP);
    EXPECT_TRUE(col_count > 0) << "Victor with box obstacle has no collision";
    // EXPECT_TRUE(!is_valid) << "Victor with box obstacle is not in collision";

    
    // victor_model.updateActual(map);
    // victor_model.doVis();
    // int dummy;
    // std::cin >> dummy;
}

TEST(GpuVoxels, addMaps)
{
    GpuVoxelsVictor vm;
    gpu_voxels::GpuVoxelsSharedPtr gvl = vm.gvl;
    std::string m1 = "TestMap1";
    std::string m2 = "TestMap2";
    gvl->addMap(MT_PROBAB_VOXELMAP, m1);
    gvl->addMap(MT_PROBAB_VOXELMAP, m2);

    EXPECT_EQ(0, vm.countIntersect(m1, m2)) << "two empty maps have an intersection";
    gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                          m1, PROB_OCCUPIED, 2);

    size_t orig_box_vox = vm.countIntersect(m1, FULL_MAP);
    EXPECT_TRUE(orig_box_vox > 0) << "Inserted box, but not seeing any voxels";

    vm.getMap(m2)->add(vm.getMap(m1));
    size_t new_box_vox = vm.countIntersect(m2, FULL_MAP);
    EXPECT_TRUE(new_box_vox > 0) << "Added map, but not seeing any voxels";
    EXPECT_EQ(orig_box_vox, new_box_vox) << "Added map, but they have a different number of voxels";
    EXPECT_EQ(orig_box_vox, vm.countIntersect(m1, m2)) << "added map not identical";

    gvl->clearMap(m2);
    EXPECT_EQ(0, vm.countIntersect(m2, FULL_MAP)) << "clearing did not clear map 2";
    vm.getMap(m2)->add(vm.getMap(m1));
    EXPECT_EQ(orig_box_vox, vm.countIntersect(m2, m1)) << "re-adding map did not work properly";
    vm.getMap(m2)->add(vm.getMap(m1));
    EXPECT_EQ(orig_box_vox, vm.countIntersect(m2, m1)) << "re-adding map did not work properly";
}



GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

