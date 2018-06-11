#include "gpu_voxels_victor.hpp"
#include <gtest/gtest.h>
#include <iostream>

#include "victor_validator.hpp"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


#define PROB_OCCUPIED eBVM_OCCUPIED
#define COLLISION_SET "chs_0"

namespace ob = ompl::base;
namespace og = ompl::geometric;


TEST(GpuVoxelVictor, collisions)
{

    std::shared_ptr<ompl::base::RealVectorStateSpace> space =
        std::make_shared<ob::RealVectorStateSpace>(7);

    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
    space->setBounds(bounds);
    std::shared_ptr<ompl::base::SpaceInformation> si_ =
        std::make_shared<ob::SpaceInformation>(space);

    GpuVoxelsVictor victor_model;
    double angles[] = {0,0,0,0,0,0,0};
    robot::JointValueMap config = victor_model.toVictorConfig(angles);
    bool is_valid = victor_model.queryFreeConfiguration(config);
    size_t col_count = victor_model.countTotalCHSCollisionsForConfig(config);
    EXPECT_EQ(0, col_count) << "Victor at initial position has collisions";
    EXPECT_TRUE(is_valid) << "Victor at initial position is in collision";


    victor_model.gvl->insertBoxIntoMap(Vector3f(1.0,0.8,1.0), Vector3f(2.0,1.0,1.2),
                                       COLLISION_SET, PROB_OCCUPIED, 2);
    victor_model.num_observed_chs = 1;


    // is_valid = victor_model.queryFreeConfiguration(config);
    // col_count = victor_model.countTotalCHSCollisionsForConfig(config);
    victor_model.resetQuery();
    victor_model.addQueryState(config);
    col_count = victor_model.countNumCollisions(COLLISION_SET);
    EXPECT_TRUE(col_count > 0) << "Victor with box obstacle has no collision";


    col_count = victor_model.countTotalCHSCollisionsForConfig(config);
    EXPECT_TRUE(col_count > 0) << "countTotalCHSCollisionsForConfig did not find a collision";



    ob::RealVectorStateSpace::StateType *test =
        si_->allocState()->as<ob::RealVectorStateSpace::StateType>();
    for(int i=0; i<7; i++)
    {
        test->values[i] = 0;
    }


    VictorValidator vv = VictorValidator(si_, &victor_model);
    VictorConservativeValidator vv_cons = VictorConservativeValidator(si_, &victor_model);
    VictorStateThresholdValidator vv_thresh = VictorStateThresholdValidator(si_, &victor_model);

    EXPECT_TRUE(vv.isValid(test)) << "Basic validator failed unexpectedly";
    EXPECT_TRUE(!vv_cons.isValid(test)) << "Conservative validator does not report collision";
    
    vv_thresh.setCostThreshold(1.0);
    EXPECT_TRUE(vv_thresh.isValid(test)) << "Even with threshold=1 there was a collision";
    
    vv_thresh.setCostThreshold(0.01);
    EXPECT_TRUE(!vv_thresh.isValid(test)) << "Even with threshold=.01 there was no collision";


    victor_model.updateActual(config);
    victor_model.doVis();
    int dummy;
    std::cin >> dummy;
}



GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

