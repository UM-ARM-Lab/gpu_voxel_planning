//
// Created by bsaund on 4/20/21.
//

#ifndef GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP
#define GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP

#include "gpu_voxel_planning/strategies/graph_search_strategies.hpp"

namespace GVP {
class OptimismIG : public GraphSearchStrategy {
  std::vector<std::shared_ptr<Goal>> generic_goals;

 public:
  explicit OptimismIG(const std::string &filename);

  OptimismIG() = default;

  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;

  [[nodiscard]] std::string getName() const override;

  void updateGoals(const Scenario &scenario) override;

  std::vector<NodeIndex> plan(NodeIndex start, std::vector<NodeIndex> goals, State &s,
                              GpuVoxelRvizVisualizer &viz) override;

  Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer &viz) override;

  Path maxIGAction(Scenario &scenario, GpuVoxelRvizVisualizer &viz);

  double calcIG(Scenario &scenario, const arc_dijkstras::GraphEdge &e) const;
};
}  // namespace GVP

#endif  // GPU_VOXEL_PLANNING_CONTACT_SHAPE_COMPLETION_STRATEGIES_HPP
