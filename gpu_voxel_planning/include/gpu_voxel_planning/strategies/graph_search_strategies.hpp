#ifndef GRAPH_SEARCH_STRATEGIES_HPP
#define GRAPH_SEARCH_STRATEGIES_HPP

#include <arc_utilities/timing.hpp>
#include <cmath>
#include <graph_planner/dijkstras_addons.hpp>

#include "gpu_voxel_planning/path_utils_addons.hpp"
#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "gpu_voxel_planning/state.hpp"
#include "gpu_voxel_planning/strategies/memorized_swept_volumes.hpp"
#include "gpu_voxel_planning/strategies/strategies.hpp"
#include "gpu_voxel_planning/strategies/victor_halton_roadmap.hpp"

namespace GVP {

typedef int64_t NodeIndex;

class GraphSearchStrategy : public Strategy {
 public:
  Roadmap graph;
  NodeIndex prev_node{};
  NodeIndex cur_node{};
  NodeIndex goal_node{};
  bool initialized;
  double discretization = 0.02;
  const std::string graph_filepath;
  const std::string swept_volumes_filepath;

  // std::map<arc_dijkstras::HashableEdge, SparseGrid> precomputed_swept_volumes;
  MemorizedSweptVolume precomputed_swept_volumes;

  // GraphSearchStrategy(const std::string &filename) : graph(filename), initialized(false) {}
  GraphSearchStrategy(const std::string &graph_filepath, const std::string &swept_volumes_filepath);
  explicit GraphSearchStrategy(const std::string &graph_filepath);
  GraphSearchStrategy();

  virtual ~GraphSearchStrategy() = default;

  virtual double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) = 0;

  Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer &viz) override;

  virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s, GpuVoxelRvizVisualizer &viz);

  void initialize(const Scenario &scenario);

  virtual DenseGrid getSweptVolume(State &s, const arc_dijkstras::GraphEdge &e);

  /* Checks an edge against known obstacles to see if there is a collision.
   * Changes the graph edge validity accordingly
   */
  bool checkEdge(arc_dijkstras::GraphEdge &e, State &s);

  double evaluateEdge(arc_dijkstras::GraphEdge &e, State &s);

  void saveToFile(const std::string& filename);
  void saveToFile() { saveToFile(swept_volumes_filepath); }

 protected:
  DenseGrid computeSweptVolume(State &s, const arc_dijkstras::GraphEdge &e);
  void storeSweptVolume(const arc_dijkstras::GraphEdge &e, const DenseGrid &g);
  std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s, Roadmap &rm);
};

class OmniscientGraphSearch : public GraphSearchStrategy {
 public:
  explicit OmniscientGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}
  OmniscientGraphSearch() = default;
  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
  std::string getName() const override;
  Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer &viz) override;
};

class OptimisticGraphSearch : public GraphSearchStrategy {
 public:
  explicit OptimisticGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}
  OptimisticGraphSearch() = default;

  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
  std::string getName() const override;
};

class ParetoCostGraphSearch : public GraphSearchStrategy {
 public:
  double alpha = 10.0;
  explicit ParetoCostGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}
  explicit ParetoCostGraphSearch(double alpha) : alpha(alpha) {}

  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
  std::string getName() const override;
};

//class UnknownSpaceCostGraphSearch : public GraphSearchStrategy {
// public:
//  double alpha;
//  double free_cost;
//  UnknownSpaceCostGraphSearch(double alpha, double free_cost) : alpha(alpha), free_cost(free_cost) {}
//
//  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
//  std::string getName() const override;
//};

class AStarGraphSearch : public GraphSearchStrategy {
 public:
  AStarGraphSearch() = default;

  std::string getName() const override;
  std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s, GpuVoxelRvizVisualizer &viz) override;
  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
  DenseGrid getSweptVolume(State &s, const arc_dijkstras::GraphEdge &e) override;
};

class ThompsonGraphSearch : public GraphSearchStrategy {
 public:
  ThompsonGraphSearch() = default;
  std::string getName() const override;

  //        State sampleValidState();

  std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s, GpuVoxelRvizVisualizer &viz) override;
  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
};

class HOPGraphSearch : public GraphSearchStrategy {
 public:
  int num_samples;

 public:
  HOPGraphSearch() : num_samples(100) {}
  std::string getName() const override;
  std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s, GpuVoxelRvizVisualizer &viz) override;
  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;
};

class ROGraphSearch : public GraphSearchStrategy {
 public:
  int num_samples;

 public:
  ROGraphSearch() : num_samples(10) {}

  bool pathExists(NodeIndex start, NodeIndex goal, State &s);
  std::vector<NodeIndex> getPossibleActions(State &state, NodeIndex cur, GpuVoxelRvizVisualizer &viz);
  double simulateTransition(State &state, const Roadmap &rm, const DenseGrid &occupied, NodeIndex &cur, NodeIndex next,
                            arc_dijkstras::EvaluatedEdges &additional_invalid, GpuVoxelRvizVisualizer &viz);

  std::vector<NodeIndex> lazySpForRollout(NodeIndex start, NodeIndex goal, State &s, Roadmap &rm,
                                          arc_dijkstras::EvaluatedEdges &additional_invalid);

  std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s, GpuVoxelRvizVisualizer &viz) override;
  double calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e) override;

  virtual double rollout(State &state, Roadmap &rm, const DenseGrid &occupied, NodeIndex cur, NodeIndex goal,
                         arc_dijkstras::EvaluatedEdges &invalid_edges_during_rollout,
                         arc_dijkstras::EvaluatedEdges &invalid_edges_during_sample, GpuVoxelRvizVisualizer &viz) = 0;
};

class OROGraphSearch : public ROGraphSearch {
  std::string getName() const override;

  double rollout(State &state, Roadmap &rm, const DenseGrid &occupied, NodeIndex cur, NodeIndex goal,
                 arc_dijkstras::EvaluatedEdges &invalid_edges_during_rollout,
                 arc_dijkstras::EvaluatedEdges &invalid_edges_during_sample, GpuVoxelRvizVisualizer &viz) override;
};

class QMDP : public ROGraphSearch {
  std::string getName() const override;

  double rollout(State &state, Roadmap &rm, const DenseGrid &occupied, NodeIndex cur, NodeIndex goal,
                 arc_dijkstras::EvaluatedEdges &invalid_edges_during_rollout,
                 arc_dijkstras::EvaluatedEdges &invalid_edges_during_sample, GpuVoxelRvizVisualizer &viz) override;
};
}  // namespace GVP

#endif
