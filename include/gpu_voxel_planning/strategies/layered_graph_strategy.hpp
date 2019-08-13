#ifndef LAYERED_GRAPH_STRATEGY_HPP
#define LAYERED_GRAPH_STRATEGY_HPP

#include "strategies/victor_selective_densification.hpp"
#include "strategies/strategies.hpp"
#include "strategies/memorized_swept_volumes.hpp"
#include <arc_utilities/timing.hpp>
#include "sd_params.hpp"


namespace GVP
{
    typedef int64_t NodeIndex;

    class LayeredGraphStrategy : public Strategy
    {
    };
    
}





#endif //LAYERED_GRAPH_STRATEGY_HPP
