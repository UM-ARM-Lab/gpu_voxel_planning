#ifndef VICTOR_TRRT_HPP
#define VICTOR_TRRT_HPP

#include "victor_planning.hpp"
#include <ompl/geometric/planners/rrt/TRRT.h>

namespace gpu_voxels_planner
{
    class VictorTrrt: public VictorPlanner
    {
    public:
        VictorTrrt(std::shared_ptr<GpuVoxelsVictor> victor_model);
        virtual void setup_planner() override;

        virtual void prepare_planner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal) override;
        virtual void post_planning_actions(ompl::base::PathPtr path) override;
    };
}

#endif
