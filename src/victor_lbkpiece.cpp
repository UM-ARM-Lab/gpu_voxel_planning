#include "victor_lbkpiece.hpp"


using namespace gpu_voxels_planner;
namespace ob = ompl::base;
namespace og = ompl::geometric;



VictorLBKPiece::VictorLBKPiece()
{
    setup_planner();
}

void VictorLBKPiece::setup_planner()
{
    planner_ = std::make_shared<og::LBKPIECE1>(si_);
}

void VictorLBKPiece::prepare_planner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
}


