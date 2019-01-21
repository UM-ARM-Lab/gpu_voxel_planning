

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include "prob_map.hpp"
#include "state.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>


#define PROB_OCCUPIED eBVM_OCCUPIED


TEST(GVP, prob_grid_copy_constructor)
{
    ProbGrid g1;
    ProbGrid g2(g1);

    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                          Vector3f(2.0,1.0,1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box, PROB_OCCUPIED);
    EXPECT_TRUE(g1.countOccupied() > 0) << "Added box, but no occupied voxels in grid";
    EXPECT_TRUE(g2.countOccupied() == 0) << "Added box to base grid, but it effected copied grid";
    EXPECT_TRUE(g1.collideWith(&g2) == 0) << "Collided an empty voxel map but found collisions. Copy did not work properly";
}


TEST(GVP, prob_grid_assignment)
{
    ProbGrid g1;
    ProbGrid g2;

    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                          Vector3f(2.0,1.0,1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box, PROB_OCCUPIED);

    g2 = g1;
    size_t occ = g1.countOccupied(); 
    EXPECT_TRUE(occ > 0) << "Added box, but no occupied voxels in grid";
    EXPECT_EQ(occ, g1.collideWith(&g2)) << "ProbGrid made with assignment operator has different number of occupied voxels";
    EXPECT_EQ(occ, g2.countOccupied()) << "assigned map has wrong number of voxels";

    g1.clearMap();
    EXPECT_EQ(0, g1.countOccupied()) << "Clearing the map did not clear the voxel grid";
    EXPECT_EQ(occ, g2.countOccupied()) << "Clearing map 1 affected map 2";
}

TEST(GVP, prob_grid_get_occupied_indices)
{
    ProbGrid g;
    Vector3f lower_left(Vector3f(1.0,0.8,1.0));
    Vector3f upper_right(Vector3f(2.0,1.0,1.2));
    PointCloud box(geometry_generation::createBoxOfPoints(lower_left,
                                                          upper_right,
                                                          VOXEL_SIDE_LENGTH/2));
    g.insertPointCloud(box, PROB_OCCUPIED);

    EXPECT_TRUE(g.countOccupied() > 0);
    EXPECT_TRUE(g.countOccupied() == g.getOccupiedCenters().size());

    for(auto oc: g.getOccupiedCenters())
    {
        EXPECT_TRUE(oc.x > lower_left.x);
        EXPECT_TRUE(oc.y > lower_left.y);
        EXPECT_TRUE(oc.z > lower_left.z);
        EXPECT_TRUE(oc.x < upper_right.x);
        EXPECT_TRUE(oc.y < upper_right.y);
        EXPECT_TRUE(oc.z < upper_right.z);
    }
}

TEST(GVP, cur_config)
{
    GVP::VictorRightArm v;
    GVP::State s(v);
    GVP::VictorRightArmConfig q(std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
    ProbGrid true_world;
    s.move(q, true_world);
    EXPECT_EQ(q, s.getCurConfig()) << "Current config not equal to config of last most";
    
}


TEST(GVP, CHS)
{
    GVP::VictorRightArm v;
    GVP::State s(v);
    GVP::VictorRightArmConfig q(std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
    ProbGrid true_world;

    EXPECT_EQ(1.0, s.calcProbFree(q)) << "No obstacles, but config is not free";
    
    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(0.0,0.0,0.0),
                                                          Vector3f(4.0,4.0,4.0),
                                                          VOXEL_SIDE_LENGTH/2));
    ProbGrid full_world;
    full_world.insertPointCloud(box, PROB_OCCUPIED);
    s.chs.push_back(full_world);

    EXPECT_GT(s.calcProbFree(q), 0.0) << "Large CHS, but no chance of free space";
    EXPECT_LT(s.calcProbFree(q), 1.0) << "Robot in CHS, but still 100% of free";
    s.move(q, true_world);
    EXPECT_EQ(s.calcProbFree(q), 1.0) << "Probability of current configuration free is 0";


    s.chs.push_back(s.robot.occupied_space);
    EXPECT_EQ(s.calcProbFree(q), 0.0) << "Robot entirely overlaps CHS, so probfree should be 0";
}






GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    icl_core::logging::setLogLevel(icl_core::logging::LogLevel::eLL_ERROR);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

