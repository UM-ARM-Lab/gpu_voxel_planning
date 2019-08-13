#ifndef ITERATIVE_DEEPENING_STRATEGIES_HPP
#define ITERATIVE_DEEPENING_STRATEGIES_HPP

#include "strategies/victor_selective_densification.hpp"
#include "strategies/strategies.hpp"
#include "strategies/memorized_swept_volumes.hpp"
#include <arc_utilities/timing.hpp>
#include "sd_params.hpp"




namespace GVP
{
    class IterativeDeepningStrategy : public Strategy
    {
    public:
        IDRoadmap id_graph;
        MemorizedSweptVolume precomputed_swept_volumes;
        bool initialized;
        double discretization = SD_EDGE_DISCRETIZATION;
        const std::string graph_filepath;
        const std::string swept_volumes_filepath;
        
    };
}



#endif //ITERATIVE_DEEPENING_STRATEGIES_HPP
