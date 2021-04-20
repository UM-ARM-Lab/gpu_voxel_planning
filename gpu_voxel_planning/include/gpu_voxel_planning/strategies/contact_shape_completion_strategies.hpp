//
// Created by bsaund on 4/20/21.
//

#ifndef GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP
#define GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP

#include "gpu_voxel_planning/strategies/graph_search_strategies.hpp"

namespace GVP {
class OptimismIG : public GraphSearchStrategy {
 public:
  explicit OptimismIG(const std::string &filename);

  OptimismIG() = default;

  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;

  std::string getName() const override;
};
}  // namespace GVP

#endif  // GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP
