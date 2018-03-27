#include "victor_lbkpiece.hpp"


using namespace gpu_voxels_planner;
namespace ob = ompl::base;
namespace og = ompl::geometric;



VictorLBKPiece::VictorLBKPiece(std::shared_ptr<GpuVoxelsVictor> victor_model)
    : VictorPlanner(victor_model)
{
    setup_planner();
}

void VictorLBKPiece::setup_planner()
{
    planner_ = std::make_shared<og::LBKPIECE1>(si_);
    planner_->setup();
}

void VictorLBKPiece::prepare_planner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    vv_ptr->insertStartAndGoal(start, goal);
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goal);

    planner_->setProblemDefinition(pdef_);

}


