#ifndef VICTOR_RRTSTAR_HPP
#define VICTOR_RRTSTAR_HPP

#include "victor_planning.hpp"
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace gpu_voxels_planner
{
    class VictorRrtStar: public VictorPlanner
    {
    public:
        VictorRrtStar(std::shared_ptr<GpuVoxelsVictor> victor_model);
        virtual void setup_planner();
        
        virtual void prepare_planner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
        virtual void post_planning_actions(ompl::base::PathPtr path);

        virtual void setupSpaceInformation();
    };
}

#endif
