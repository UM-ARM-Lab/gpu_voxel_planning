

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include "maps/prob_map.hpp"
#include "maps/distance_map.hpp"
#include "obstacles/obstacles.hpp"
#include "state.hpp"
#include "beliefs/beliefs.hpp"
#include "strategies/memorized_swept_volumes.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>


#define PROB_OCCUPIED eBVM_OCCUPIED


TEST(GVP, dense_grid_copy_constructor)
{
    DenseGrid g1;
    DenseGrid g2(g1);

    g1.insertBox(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2));
    EXPECT_TRUE(g1.countOccupied() > 0) << "Added box, but no occupied voxels in grid";
    EXPECT_TRUE(g2.countOccupied() == 0) << "Added box to base grid, but it effected copied grid";
    EXPECT_TRUE(g1.collideWith(&g2) == 0) << "Collided an empty voxel map but found collisions. Copy did not work properly";
}


TEST(GVP, dense_grid_assignment)
{
    DenseGrid g1;
    DenseGrid g2;

    g1.insertBox(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2));


    g2 = g1;
    size_t occ = g1.countOccupied(); 
    EXPECT_TRUE(occ > 0) << "Added box, but no occupied voxels in grid";
    EXPECT_EQ(occ, g1.collideWith(&g2)) << "DenseGrid made with assignment operator has different number of occupied voxels";
    EXPECT_EQ(occ, g2.countOccupied()) << "assigned map has wrong number of voxels";

    g1.clearMap();
    EXPECT_EQ(0, g1.countOccupied()) << "Clearing the map did not clear the voxel grid";
    EXPECT_EQ(occ, g2.countOccupied()) << "Clearing map 1 affected map 2";
}

TEST(GVP, dense_grid_get_occupied_indices)
{
    DenseGrid g;
    Vector3f upper_right(2.0,1.0,1.2);
    Vector3f lower_left(1.0,0.8,1.0);
    g.insertBox(lower_left, upper_right);


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

TEST(GVP, dense_grid_copyRandomOccupiedElement)
{
    DenseGrid from1, from2;
    from1.insertBox(Vector3f(1.0, 0.8, 0.7), Vector3f(2.0, 1.0, 1.0));
    from2.insertBox(Vector3f(0.5, 0.8, 0.7), Vector3f(1.5, 1.0, 1.0));

    for(int i=0; i<30; i++)
    {
        DenseGrid to;
        from1.copyRandomOccupiedElement(to);
        ASSERT_EQ(from1.collideWith(&to), 1) << "Not one voxel of collision after copy random element";
                
        from2.copyRandomOccupiedElement(to);

        ASSERT_TRUE(from1.collideWith(&to)) << "from1 did not collide after copying from from2";
        ASSERT_TRUE(from2.collideWith(&to)) << "from2 did not collide after copying from from2";
    }
}

// TEST(GVP, cur_config)
// {
//     GVP::VictorRightArm v;
//     GVP::SimulationState s(v);
//     GVP::VictorRightArmConfig q(std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
//     DenseGrid true_world;
//     s.move(q, true_world);
//     EXPECT_EQ(q, s.getCurConfig()) << "Current config not equal to config of last most";
    
// }


TEST(GVP, CHS_probabilities)
{
    GVP::VictorRightArm robot;
    // GVP::SimulationState s(v);
    GVP::ChsBelief bel;
    GVP::VictorRightArmConfig q(std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
    DenseGrid true_world;

    robot.set(q.asMap());
    DenseGrid robot_volume = robot.occupied_space;

    EXPECT_EQ(1.0, bel.calcProbFree(robot_volume)) << "No obstacles, but config is not free";
 
    DenseGrid full_world;
    full_world.insertBox(Vector3f(0.0,0.0,1.0), Vector3f(4.0,4.0,4.0));

    bel.chs.push_back(full_world);

    EXPECT_GT(bel.calcProbFree(robot_volume), 0.0) << "Large CHS, but no chance of free space";
    EXPECT_LT(bel.calcProbFree(robot_volume), 1.0) << "Robot in CHS, but still 100% of free";
    // s.move(q, true_world);
    // EXPECT_EQ(bel.calcProbFree(robot_volume), 1.0) << "Probability of current configuration free is 0";


    bel.chs.push_back(robot.occupied_space);
    EXPECT_EQ(bel.calcProbFree(robot_volume), 0.0) << "Robot entirely overlaps CHS, so probfree should be 0";
}


TEST(GVP, sparse_grid_copy)
{
    DenseGrid dg0, dg1;
    dg0.insertBox(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2));
    SparseGrid sg1;
    sg1.merge(&dg0);
    dg1.merge(&sg1);

    EXPECT_EQ(dg0.countOccupied(), dg1.countOccupied()) << "Converting to sparse and back changed number of occupied voxels";
    EXPECT_EQ(dg0.countOccupied(), dg0.collideWith(&dg1)) << "Converting to sparse and back changed number of occupied voxels";

    SparseGrid sg2(dg0);
    DenseGrid dg2(sg2);
    
    EXPECT_EQ(dg0.countOccupied(), dg2.countOccupied()) << "Converting to sparse and back changed number of occupied voxels";
    EXPECT_EQ(dg0.countOccupied(), dg0.collideWith(&dg2)) << "Converting to sparse and back changed number of occupied voxels";

    SparseGrid sg3;
    sg3 = dg0;
    DenseGrid dg3(sg3);
    
    EXPECT_EQ(dg0.countOccupied(), dg3.countOccupied()) << "Converting to sparse and back changed number of occupied voxels";
    EXPECT_EQ(dg0.countOccupied(), dg0.collideWith(&dg3)) << "Converting to sparse and back changed number of occupied voxels";
    
}


TEST(GVP, file_read_write)
{
    DenseGrid dg0, dg1;
    dg0.insertBox(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2));
    SparseGrid sg0(dg0), sg1;


    sg0.writeToDisk("/tmp/sparse_grid.gvg");
    dg0.writeToDisk("/tmp/dense_grid.gvg");

    sg1.readFromDisk("/tmp/sparse_grid.gvg");
    dg1.readFromDisk("/tmp/dense_grid.gvg");

    DenseGrid dg2(sg1);

    EXPECT_EQ(dg1.countOccupied(), dg0.countOccupied()) << "Writing and Reading Dense Grid resulted in change";
    EXPECT_EQ(dg2.countOccupied(), dg0.countOccupied()) << "Writing and Reading Sparse Grid resulted in change";
}


TEST(GVP, memorize_swept_volumes_loading)
{
    DenseGrid dg1, dg2;
    dg1.insertBox(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2));
    dg2.insertBox(Vector3f(0.1,0.2,0.3), Vector3f(0.4,0.5,0.6));
    arc_dijkstras::HashableEdge e1(3, 4);
    arc_dijkstras::HashableEdge e2(7, 8);

    GVP::MemorizedSweptVolume sv;
    sv[e1] = dg1;
    sv[e2] = dg2;

    std::vector<uint8_t> buffer;
    sv.serializeSelf(buffer);

    uint64_t buffer_position = 0;
    GVP::MemorizedSweptVolume sv_loaded;
    sv_loaded.deserializeSelf(buffer, buffer_position);
        
    DenseGrid dg1p = sv_loaded[e1];
    DenseGrid dg2p = sv_loaded[e2];

    EXPECT_EQ(dg1.countOccupied(), dg1p.countOccupied());
    EXPECT_EQ(dg1.collideWith(&dg1p), dg1p.countOccupied());
    
    EXPECT_EQ(dg2.countOccupied(), dg2p.countOccupied());
    EXPECT_EQ(dg2.collideWith(&dg2p), dg2p.countOccupied());
        
}

TEST(GVP, distance_grid_gives_correct_distances)
{
    DistanceGrid dg;
    DenseGrid g1, g2;
    double eps = 0.000001;
    PointCloud box1(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                           Vector3f(2.0,1.0,1.2),
                                                           VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box1, PROB_OCCUPIED);
    dg.mergeOccupied(&g1);
    dg.computeDistances();

    PointCloud box2(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,0.5),
                                                           Vector3f(2.0,1.0,0.7),
                                                           VOXEL_SIDE_LENGTH/2));
    g2.insertPointCloud(box2, PROB_OCCUPIED);

    double d = dg.getClosestObstacleDistance(&g2)*VOXEL_SIDE_LENGTH;
    EXPECT_NEAR(d, 0.3, VOXEL_SIDE_LENGTH + eps) << "Distance not correct\n";
}

TEST(GVP, test_AABB_obstacle_collision_with_shifts)
{
    Vector3f v1(1.0, 0.8, 1.0);
    Vector3f v2(2.0, 1.0, 1.2);
    GVP::AABB ob(v1, v2);
    DenseGrid g1;
    g1.insertBox(v1, v2);

    EXPECT_EQ(ob.grid.collideWith(&g1), g1.countOccupied());
    ob.shift(Vector3f(0.1, 0.1, 0.1));
    EXPECT_LT(ob.grid.collideWith(&g1), g1.countOccupied());
    EXPECT_GT(ob.grid.collideWith(&g1), 0);
}

TEST(GVP, test_AABB_projection)
{
    double eps = 0.0000001;
    GVP::AABB ob(Vector3f(1.0, 0.8, 1.0), Vector3f(2.0, 1.0, 1.2));
    DenseGrid g;
    g.insertBox(Vector3f(1.0, 0.8, 0.5), Vector3f(2.0, 1.0, 0.7));
    DistanceGrid dg;
    dg.mergeOccupied(&g);

    ASSERT_NEAR(dg.getClosestObstacleDistance(&ob.grid) * VOXEL_SIDE_LENGTH, 0.3, VOXEL_SIDE_LENGTH + eps) <<
        "Distance not correct before projection";

    
    ob.project(dg);
    EXPECT_NEAR(dg.getClosestObstacleDistance(&ob.grid)*VOXEL_SIDE_LENGTH, 0, VOXEL_SIDE_LENGTH + eps) <<
        "Distance not correct after projection";
}


GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    icl_core::logging::setLogLevel(icl_core::logging::LogLevel::eLL_ERROR);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

