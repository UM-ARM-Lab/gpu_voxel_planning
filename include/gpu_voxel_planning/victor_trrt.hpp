#ifndef VICTOR_TRRT_HPP
#define VICTOR_TRRT_HPP

#include "victor_planning.hpp"

namespace gpu_voxels_planner
{
    class VictorTrrt: public VictorPlanner
    {
    public:
        VictorTrrt();
        virtual void setup_planner();
        
        virtual void prepare_planner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
        virtual void post_planning_actions(ompl::base::PathPtr path);
    };
}

#endif
