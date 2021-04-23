//
// Created by bsaund on 4/23/21.
//
#include <gtest/gtest.h>
#include "gpu_voxel_planning/utils/information_utils.hpp"

using namespace GVP;

TEST(IG, test_case_of_no_inforamtion_gain)
{
  std::vector<int> measurements = {1, 1, 1, 1, 1};
  EXPECT_EQ(0.0, calcIG(measurements));
}

TEST(IG, test_maximal_IG_is_maximal)
{
  std::vector<int> best_measurements = {1, 2, 3, 4, 5};
  std::vector<int> second_best_measurements = {1, 2, 3, 4, 4};
  EXPECT_GT(calcIG(best_measurements), calcIG(second_best_measurements));
  EXPECT_GT(calcIG(best_measurements), 0);
  EXPECT_GT(calcIG(second_best_measurements), 0);
}

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}