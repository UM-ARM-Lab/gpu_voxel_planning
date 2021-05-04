//
// Created by bsaund on 4/20/21.
//

#include "gpu_voxel_planning/strategies/contact_shape_completion_strategies.hpp"
#include "gpu_voxel_planning/utils/information_utils.hpp"
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

  return GraphSearchStrategy::plan(start, goals, s, viz);
}

Path OptimismIG::maxIGAction(Scenario& scenario, GpuVoxelRvizVisualizer &viz) {
  scenario.getState().bel->viz(viz);
  const VictorRightArmConfig &cur_config(scenario.getState().getCurConfig());
  auto cur_ind = graph.getNodeAt(cur_config.asVector());
  if(!cur_ind.has_value()){
    cur_ind = graph.addVertexAndEdges(cur_config.asVector());
  }
  const auto& cur_node = graph.getNode(cur_ind.value());
  std::cout << "IG: considering " << cur_node.getOutEdges().size() << " edges\n";

  double best_IG = 0;
  const arc_dijkstras::GraphEdge* best_edge;
  for(const auto& edge: cur_node.getOutEdges()){
    viz.vizGrid(getSweptVolume(scenario.getState(), edge), "swept_edge");
    double ig = calcIG(scenario, edge);
    if(ig > best_IG){
      best_IG = ig;
      best_edge = &edge;
    }
  }
  viz.clearGrid("swept_edge");

  std::cout << "Best action has IG of " << best_IG << "\n";
  if(best_IG == 0){
    throw std::logic_error("No action has any information. Need to implement next strategy");
  }

  auto cur = VictorRightArmConfig(graph.getNode(best_edge->getFromIndex()).getValue());
  auto next = VictorRightArmConfig(graph.getNode(best_edge->getToIndex()).getValue());

  return interpolate(cur, next, discretization);
}

double OptimismIG::calcIG(Scenario &scenario, const arc_dijkstras::GraphEdge &e) const{
  auto bel = dynamic_cast<ShapeCompletionBelief*>(scenario.getState().bel.get());
  const auto& particles = bel->sampled_particles;

  VictorRightArmConfig q_start(graph.getFromValue(e));
  VictorRightArmConfig q_end(graph.getToValue(e));
  GVP::Path path = interpolate(q_start, q_end, discretization);
  auto sv = computeSweptVolume(scenario.getState(), e);

  std::vector<int> collision_points(particles.size());
  for(int i=0; i<particles.size(); i++){
    if(!sv.overlapsWith(&particles[i])){
      break;
    }

    for(int j=0; j<path.size(); j++){
      scenario.victor.set(path[j].asMap());
      if(scenario.victor.occupied_space.overlapsWith(&particles[i])){
        collision_points[i] = j;
        break;
      }
    }
  }



//  std::cout << "Collision points: " << PrettyPrint::PrettyPrint(collision_points) << "\n";

  return GVP::calcIG(collision_points);
}

Path OptimismIG::applyTo(Scenario& scenario, GpuVoxelRvizVisualizer& viz) {
  if (!initialized) {
    initialize(scenario);
  }

  updateGoals(scenario);

  const VictorRightArmConfig &current(scenario.getState().getCurConfig());
  VictorRightArmConfig expected(graph.getNode(cur_node).getValue());
  VictorRightArmConfig next;

  for(const auto& goal: scenario.getState().bel->getGoals()){
    if(goal->isAchieved(current, &scenario)){
      std::cout << "A goal is already achieved. Running IG attempt\n";
      return maxIGAction(scenario, viz);
    }
  }

  if (current == expected) {
    //TODO: Temporary. Only uses first goal node
    std::vector<NodeIndex> node_path = plan(cur_node, goal_nodes, scenario.getState(), viz);
    std::cout << "Planning to nodes: " << PrettyPrint::PrettyPrint(node_path, true) << "\n";
    next = VictorRightArmConfig(graph.getNode(node_path[1]).getValue());
    prev_node = cur_node;
    cur_node = node_path[1];
  } else {
    next = VictorRightArmConfig(graph.getNode(prev_node).getValue());
    graph.getEdge(prev_node, cur_node).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    graph.getEdge(cur_node, prev_node).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);

    cur_node = prev_node;
  }

  return interpolate(current, next, discretization);
}

