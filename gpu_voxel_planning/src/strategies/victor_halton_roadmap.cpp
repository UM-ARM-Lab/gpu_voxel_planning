#include "gpu_voxel_planning/strategies/victor_halton_roadmap.hpp"

#include <arc_utilities/timing.hpp>

#include "gpu_voxel_planning/hacky_functions.hpp"

static double getConnectionRadius(int num_nodes, int dim, double num_neighbors_desired) {
  if (dim != 7) {
    throw std::invalid_argument("radiusAtDepth only writted to accomodate dimension 7");
  }

  double num_points = num_nodes;
  double space_volume = 1.0;

  for (int i = 0; i < right_joint_lower_deg.size(); i++) {
    space_volume *= (right_joint_upper_deg[i] - right_joint_lower_deg[i]) * torad;
  }

  double connection_ball_volume = (double)num_neighbors_desired * space_volume / num_points;
  double connection_radius = 0.8 * std::pow(connection_ball_volume, 1.0 / dim);
  // https://en.wikipedia.org/wiki/Volume_of_an_n-ball

  return connection_radius;
}

Roadmap::Roadmap() : HaltonGraph(0, 0, 0) {
  // int num_vert = 1000;
  // edge_dist = 4.0;
  // int num_vert = 10000;
  // r_disc = 2.0;
  // int num_vert = 100000;
  // r_disc = 1.5;
  // int num_vert = 100000;
  // r_disc = 1.3;
  int num_vert = 1000000;
  r_disc = 1.0;

  auto configs = scaleToVictorDims(halton::haltonPoints(num_vert, 7));

  PROFILE_START("node_creation");
  int i = 0;
  for (auto q : configs) {
    addVertexAndEdges(q);
    i++;
    if (i % 10000 == 0) {
      std::cout << i << "/" << num_vert << " nodes in " << PROFILE_RECORD("node_creation") << "s\n";
    }
  }
  // V = toNodes(haltonPoints(bases, num_vert, offsets));
  // addEdges(edge_dist);

  std::cout << "Made graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}

Roadmap::Roadmap(int num_nodes, double approx_edges_per_node) : HaltonGraph(0, 0, 0) {
  r_disc = getConnectionRadius(num_nodes, 7, approx_edges_per_node);

  auto configs = scaleToVictorDims(halton::haltonPoints(num_nodes, 7));

  PROFILE_START("node_creation");
  int i = 0;
  for (auto q : configs) {
    addVertexAndEdges(q);
    i++;
    if (i % 10000 == 0) {
      std::cout << i << "/" << num_nodes << " nodes in " << PROFILE_RECORD("node_creation") << "s\n";
    }
  }
  std::cout << "Made graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}

Roadmap::Roadmap(std::string filename) : HaltonGraph(0, 0, 0) {
  loadFromFile(filename);
  std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}

std::vector<std::vector<double>> Roadmap::scaleToVictorDims(std::vector<std::vector<double>> points) {
  for (auto &point : points) {
    for (int i = 0; i < point.size(); i++) {
      double range = (right_joint_upper_deg[i] - right_joint_lower_deg[i]);
      point[i] = (point[i] * range + right_joint_lower_deg[i]) * torad;
    }
  }
  return points;
}
