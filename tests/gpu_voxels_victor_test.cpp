#include "gpu_voxels_victor.hpp"
#include <gtest/gtest.h>
#include <iostream>


#define PROB_OCCUPIED eBVM_OCCUPIED
#define ENV_MAP "env_map"

/*
 * returns true if either map1 is a subset of map2, or map2 is a subset of map 1
 */
bool isSubset(GpuVoxelsVictor &vm, const std::string &map1, const std::string &map2)
{
    size_t intersect = vm.countIntersect(map1, map2);
    size_t tot1 = vm.countVoxels(map1);
    size_t tot2 = vm.countVoxels(map2);
    return (tot1 == intersect) || (tot2 == intersect);
}

TEST(GpuVoxelVictor, collisions)
{

    GpuVoxelsVictor victor_model;
    double angles[] = {0,0,0,0,0,0,0};
    robot::JointValueMap map = victor_model.toVictorConfig(angles);
    bool is_valid = victor_model.queryFreeConfiguration(map);
    size_t col_count = victor_model.countTotalCHSCollisionsForConfig(map);
    EXPECT_EQ(0, col_count) << "Victor at initial position has collisions";
    EXPECT_TRUE(is_valid) << "Victor at initial position is in collision";


    victor_model.gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                                       ENV_MAP, PROB_OCCUPIED, 2);

    // is_valid = victor_model.queryFreeConfiguration(map);
    // col_count = victor_model.countTotalCHSCollisionsForConfig(map);
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

TEST(GpuVoxelVictor, addCHS_MultipleSets)
{
    GpuVoxelsVictor vm;
    std::vector<double> start = {0,0,0,0,0,0,0};
    std::vector<double> mid1 = {.1, .1, .1, .1, .1, .1, .1};
    std::vector<double> end1 = {.2, .2, .2, .2, .2, .2, .2};
    std::vector<double> mid2 = {.1, .5, .8, .0, .11, .9, 1.0};
    std::vector<double> end2 = {.4, .8, 1.9, -1.0, .1, 1.5, 1.9};

    Path col0{mid1, end1};
    Path col1{mid2, end2};

    vm.updateActual(vm.toVictorConfig(start.data()));

    EXPECT_EQ(0, vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0])) << "non-empty initial CHS 0";
    EXPECT_EQ(0, vm.countVoxels(COLLISION_HYPOTHESIS_SETS[1])) << "non-empty initial CHS 1";

    vm.addCHS(col0, vm.right_arm_collision_link_names);

    size_t chs0_t1 = vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0]);

    EXPECT_TRUE(chs0_t1 > 0) << "empty CHS 0 after collision";
    EXPECT_EQ(0, vm.countVoxels(COLLISION_HYPOTHESIS_SETS[1])) << "non-empty CHS 1 after first collision";

    vm.addCHS(col1, vm.right_arm_collision_link_names);
    size_t chs0_t2 = vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0]);
    size_t chs1_t2 = vm.countVoxels(COLLISION_HYPOTHESIS_SETS[1]);
    
    EXPECT_TRUE(chs0_t2 <= chs0_t1) << "CHS 0 increased in size at t2";
    EXPECT_TRUE(chs0_t2 > 0) << "CHS 0 is size 0 at t2";
    EXPECT_TRUE(chs1_t2 > 0) << "CHS 1 is size 0 at t2";
    size_t intersect = vm.countIntersect(COLLISION_HYPOTHESIS_SETS[0], COLLISION_HYPOTHESIS_SETS[1]);
    EXPECT_TRUE(intersect < chs0_t2) << "chs0 is a subset of chs1";
    EXPECT_TRUE(intersect < chs1_t2) << "chs1 is a subset of chs0";
}

TEST(GpuVoxelVictor, addCHS_PartialDistance)
{
    GpuVoxelsVictor vm;
    std::vector<double> start = {0,0,0,0,0,0,0};
    std::vector<double> end = {.9, .9, .9, .9, .9, .9, .9};
    Path path{start, end};

    vm.addCHS(path, vm.right_gripper_collision_link_names);
    EXPECT_TRUE(vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0]) > 0) << "empty chs after adding";

    vm.resetQuery();
    vm.addQueryState(vm.toVictorConfig(start.data()));

    EXPECT_TRUE(vm.countIntersect(VICTOR_QUERY_MAP, COLLISION_HYPOTHESIS_SETS[0]) > 0) << "CHS did not add voxels in the right place";

    vm.resetQuery();
    vm.addQueryState(vm.toVictorConfig(end.data()));

    EXPECT_EQ(0, vm.countIntersect(VICTOR_QUERY_MAP, COLLISION_HYPOTHESIS_SETS[0]) > 0) << "Far away config overlaps with CHS";

}

TEST(GpuVoxelVictor, hypothetical_reset)
{
    GpuVoxelsVictor vm;
    std::vector<double> start = {0,0,0,0,0,0,0};
    std::vector<double> mid = {.1, .1, .1, .1, .1, .1, .1};
    std::vector<double> end = {.2, .2, .2, .2, .2, .2, .2};

    vm.updateActual(vm.toVictorConfig(start.data()));

    Path col_path{mid, end};

    vm.addCHS(col_path, vm.right_arm_collision_link_names);

    EXPECT_TRUE(vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0]) > 0);

    // std::string unused;
    // std::getline(std::cin, unused);
    
    vm.gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                             ENV_MAP, PROB_OCCUPIED, 2);
    vm.resetHypothetical();
    EXPECT_TRUE(vm.countVoxels(VICTOR_SWEPT_VOLUME_MAP) > 0) << "freespace is empty";
    EXPECT_TRUE(vm.countVoxels(HFREE) > 0) << "hypothetical free space is empty";
    EXPECT_EQ(vm.countVoxels(VICTOR_SWEPT_VOLUME_MAP), vm.countIntersect(VICTOR_SWEPT_VOLUME_MAP, HFREE)) << "freespace and hypothetical free space are not equal after initialization";

    EXPECT_EQ(vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0]), vm.countIntersect(COLLISION_HYPOTHESIS_SETS[0], HCHS[0])) << "hypothetical and original chs are not equal";

    vm.resetQuery();
    vm.addQueryState(vm.toVictorConfig(mid.data()));
    vm.getMap(HFREE)->add(vm.getMap(VICTOR_QUERY_MAP));
    vm.getMap(HCHS[0])->subtract(vm.getMap(HFREE));
    
    EXPECT_TRUE(vm.countVoxels(HFREE) > vm.countVoxels(VICTOR_SWEPT_VOLUME_MAP)) << "hypothtical free is not larger than original free after adding a new config";
    EXPECT_EQ(vm.countVoxels(VICTOR_SWEPT_VOLUME_MAP), vm.countIntersect(VICTOR_SWEPT_VOLUME_MAP, HFREE)) << "hfree is not a superset of original freespace";

    EXPECT_TRUE(vm.countVoxels(HCHS[0]) < vm.countVoxels(COLLISION_HYPOTHESIS_SETS[0])) << "hypothtical chs is not smaller that original after adding config";
    EXPECT_EQ(vm.countVoxels(HCHS[0]), vm.countIntersect(HCHS[0], COLLISION_HYPOTHESIS_SETS[0])) << "HCHS[0] is not a subset of original chs";
}



TEST(GpuVoxels, addMaps)
{
    GpuVoxelsVictor vm;
    gpu_voxels::GpuVoxelsSharedPtr gvl = vm.gvl;
    std::string m1 = "TestMap1";
    std::string m2 = "TestMap2";
    gvl->addMap(MT_PROBAB_VOXELMAP, m1);
    gvl->addMap(MT_PROBAB_VOXELMAP, m2);

    EXPECT_EQ(0, vm.countIntersect(m1, m2)) << "two empty maps have a non-empty intersection";
    gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                          m1, PROB_OCCUPIED, 2);

    size_t orig_box_vox = vm.countVoxels(m1);
    EXPECT_TRUE(orig_box_vox > 0) << "Inserted box, but not seeing any voxels";

    vm.getMap(m2)->add(vm.getMap(m1));
    size_t new_box_vox = vm.countVoxels(m2);
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
    icl_core::logging::setLogLevel(icl_core::logging::LogLevel::eLL_ERROR);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

