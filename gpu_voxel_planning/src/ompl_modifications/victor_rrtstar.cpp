#include "victor_rrtstar.hpp"

#include "optimization_objectives.hpp"
#include "wip_optimization_objective.hpp"

using namespace gpu_voxels_planner;
namespace ob = ompl::base;
namespace og = ompl::geometric;

VictorRrtStar::VictorRrtStar(std::shared_ptr<GpuVoxelsVictor> victor_model) : VictorPlanner(victor_model) {
  objective_ = std::make_shared<WipOptimizationObjective>(si_, victor_model);
  setup_planner();
}

void VictorRrtStar::setup_planner() {
  setupSpaceInformation();

  planner_ = std::make_shared<og::RRTstar>(si_);

  // pdef_->setOptimizationObjective(objective_);
}

void VictorRrtStar::setupSpaceInformation() {
  std::cout << "calling dummy setupSpaceInformation\n";
  // si_->setStateValidityChecker(vv_ptr->getptr());
  // si_->setMotionValidator(vv_ptr->getptr());
  si_->setup();
}

void VictorRrtStar::prepare_planner(ob::ScopedState<> start, ob::ScopedState<> goal) {
  planner_->clear();
  vv_ptr->insertStartAndGoal(start, goal);
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goal);

  pdef_->setOptimizationObjective(objective_);
  planner_->setProblemDefinition(pdef_);
  planner_->setup();

  // ob::GoalPtr goal_ptr = ((og::TRRT*)(planner_.get()) )->getGoal();
}

void VictorRrtStar::post_planning_actions(ompl::base::PathPtr path) {
  og::PathGeometric *sol = path->as<ompl::geometric::PathGeometric>();
  std::cout << "Path starts at:\n ";
  const double *start_values = sol->getState(0)->as<ob::RealVectorStateSpace::StateType>()->values;
  for (int i = 0; i < 7; i++) {
    std::cout << start_values[i] << ", ";
  }
  std::cout << "\n";

  std::cout << "Path ends at:\n ";

  const double *end_values = sol->getState(sol->getStateCount() - 1)->as<ob::RealVectorStateSpace::StateType>()->values;
  for (int i = 0; i < 7; i++) {
    std::cout << end_values[i] << ", ";
  }
  std::cout << "\n";
}
