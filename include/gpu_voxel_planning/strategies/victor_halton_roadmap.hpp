#ifndef VICTOR_HALTON_ROADMAP_HPP
#define VICTOR_HALTON_ROADMAP_HPP

// #include <graph_planner/graph.hpp>
// #include <graph_planner/halton.hpp>
// #include <graph_planner/a_star.hpp>
#include <graph_planner/halton_graph.hpp>



class Roadmap : public HaltonGraph
{
public:
    
    Roadmap();
    Roadmap(int num_nodes, double approx_edges_per_node);
    Roadmap(std::string filename);
    std::vector<std::vector<double>> scaleToVictorDims(std::vector<std::vector<double>> points);
};



#endif
