#include "wip_optimization_objective.hpp"

namespace ob = ompl::base;
// namespace og = ompl::geometric;

WipOptimizationObjective::WipOptimizationObjective(ompl::base::SpaceInformationPtr si,
                                                   gpu_voxels::GpuVoxelsSharedPtr gvl)
    : ob::OptimizationObjective(si)
{
    gvl_ = gvl;
}


ob::Cost WipOptimizationObjective::stateCost(const ob::State *s) const
{
    return ob::Cost();
}

ob::Cost WipOptimizationObjective::motionCost(const ob::State *s1,
                                              const ob::State *s2) const
{
    return ob::Cost();
}
     
