#ifndef VICTOR_PLANNING_HPP
#define VICTOR_PLANNING_HPP

#include "victor_validator.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

namespace gpu_voxels_planner
{
    class VictorPlanner
    {
    public:
        VictorPlanner();
        ompl::base::PathPtr planPath(ompl::base::ScopedState<> start,
                                     ompl::base::ScopedState<> goal);
        ompl::base::PathPtr planPath(std::vector<float> start, std::vector<float> goal);
        std::shared_ptr<VictorValidator> vv_ptr;
        std::shared_ptr<ompl::base::RealVectorStateSpace> space;
    private:

        std::shared_ptr<ompl::base::SpaceInformation> si;
        std::shared_ptr<ompl::geometric::PathSimplifier> simp;
        std::shared_ptr<ompl::base::ProblemDefinition> pdef;
        std::shared_ptr<ompl::geometric::LBKPIECE1> planner;
    };
}
        
#endif
