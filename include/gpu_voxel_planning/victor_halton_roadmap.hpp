#ifndef VICTOR_HALTON_ROADMAP_HPP
#define VICTOR_HALTON_ROADMAP_HPP

// #include <graph_planner/graph.hpp>
// #include <graph_planner/halton.hpp>
// #include <graph_planner/a_star.hpp>
#include <graph_planner/halton_graph.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "gpu_voxels_victor.hpp"




class Roadmap : public HaltonGraph
{
public:
    GpuVoxelsVictor* victor_;

    
    Roadmap(GpuVoxelsVictor* victor);
    Roadmap(GpuVoxelsVictor* victor, std::string filename);
    std::vector<std::vector<double>> scaleToVictorDims(std::vector<std::vector<double>> points);
};



#endif
