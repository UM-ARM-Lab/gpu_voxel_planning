#ifndef VICTOR_HALTON_ROADMAP_HPP
#define VICTOR_HALTON_ROADMAP_HPP

#include <graph_planner/graph.hpp>
#include <graph_planner/halton.hpp>
#include <graph_planner/a_star.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "gpu_voxels_victor.hpp"




class Roadmap : public Graph
{
public:
    GpuVoxelsVictor* victor_;

    
    Roadmap(GpuVoxelsVictor* victor);
    std::vector<std::vector<double>> scaleToVictorDims(std::vector<std::vector<double>> points);
};



#endif
