#include <gtest/gtest.h>
#include "strategies/graph_search_strategies.hpp"
#include "mock/mock_gpu_voxel_rviz_visualization.hpp"

using namespace GVP;

TEST(GS_Strat, ORO_repeat)
{
    BeliefParams bp(BeliefType::CHS);
    MockGpuVoxelRvizVisualizer viz;

    OptimisticGraphSearch strat;

    TableWithBox scenario(bp, true, true, false);

    auto path = strat.applyTo(scenario, viz);
}


GTEST_API_ int main(int argc, char **argv) {
    icl_core::logging::initialize(argc, argv);
    icl_core::logging::setLogLevel(icl_core::logging::LogLevel::eLL_ERROR);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
