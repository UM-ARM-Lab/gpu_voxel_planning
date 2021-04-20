//
// Created by bsaund on 4/20/21.
//

#include "gpu_voxel_planning/strategies/contact_shape_completion_strategies.hpp"
using namespace GVP;

OptimismIG::OptimismIG(const std::string& filename) : GraphSearchStrategy(filename) {}

double OptimismIG::calculateEdgeWeight(State& s, const arc_dijkstras::GraphEdge& e) {
  if (s.calcProbFree(getSweptVolume(s, e)) > 0) {
    return e.getWeight();
  }
  return std::numeric_limits<double>::infinity();
}

std::string OptimismIG::getName() const { return "OptimismIG"; }

void OptimismIG::updateGoals(const Scenario& scenario) {
  GraphSearchStrategy::updateGoals(scenario);
  generic_goals = scenario.getState().bel->getGoals();
  //  auto bel = scenario.getState().bel.get();
  //  auto shape_bel = dynamic_cast<ShapeCompletionBelief*>(bel);
  //  shape_bel->
}


std::vector<NodeIndex> OptimismIG::plan(NodeIndex start, std::vector<NodeIndex> goals, State& s,
                                        GpuVoxelRvizVisualizer& viz) {
  for(const auto& goal: generic_goals){
    goal.
  }

  return GraphSearchStrategy::plan(start, goals, s, viz);
}
