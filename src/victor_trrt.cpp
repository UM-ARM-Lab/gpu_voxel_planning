#include "victor_trrt.hpp"


using namespace gpu_voxels_planner;
namespace ob = ompl::base;
namespace og = ompl::geometric;



VictorTrrt::VictorTrrt()
{
    setup_planner();
}

void VictorTrrt::setup_planner()
{
    planner_ = std::make_shared<og::LBKPIECE1>(si_);
    planner_->setup();
}

void VictorTrrt::prepare_planner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner_->clear();
    vv_ptr->insertStartAndGoal(start, goal);
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goal);

    planner_->setProblemDefinition(pdef_);

}


void VictorTrrt::post_planning_actions(ompl::base::PathPtr path)
{
    og::PathGeometric* sol = path->as<ompl::geometric::PathGeometric>();
    std::cout << "Path starts at " << sol->getState(0)->as<ob::RealVectorStateSpace::StateType>()->values << "\n";
    std::cout << "Path ends at " << sol->getState(sol->getStateCount()-1)->as<ob::RealVectorStateSpace::StateType>()->values << "\n";
}
