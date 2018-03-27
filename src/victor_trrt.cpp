#include "victor_trrt.hpp"


using namespace gpu_voxels_planner;
namespace ob = ompl::base;
namespace og = ompl::geometric;



VictorTrrt::VictorTrrt(std::shared_ptr<GpuVoxelsVictor> victor_model)
    : VictorPlanner(victor_model)
{
    setup_planner();
}

void VictorTrrt::setup_planner()
{
    planner_ = std::make_shared<og::TRRT>(si_);
    planner_->setup();
}

void VictorTrrt::prepare_planner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner_->clear();
    vv_ptr->insertStartAndGoal(start, goal);
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goal);

    planner_->setProblemDefinition(pdef_);
    // ob::GoalPtr goal_ptr = ((og::TRRT*)(planner_.get()) )->getGoal();

}


void VictorTrrt::post_planning_actions(ompl::base::PathPtr path)
{
    og::PathGeometric* sol = path->as<ompl::geometric::PathGeometric>();
    std::cout << "Path starts at:\n ";
    const double *start_values = sol->getState(0)->as<ob::RealVectorStateSpace::StateType>()->values;
    for(int i=0; i<7; i++)
    {
        std::cout << start_values[i] << ", ";
    }
    std::cout << "\n";

    std::cout << "Path ends at:\n ";
    
    const double *end_values = sol->getState(sol->getStateCount()-1)->as<ob::RealVectorStateSpace::StateType>()->values;
    for(int i=0; i<7; i++)
    {
        std::cout << end_values[i] << ", ";
    }
    std::cout << "\n";
    
    
}
