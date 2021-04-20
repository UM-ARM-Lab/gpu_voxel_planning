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
