#include "wip_optimization_objective.hpp"

namespace ob = ompl::base;
// namespace og = ompl::geometric;

WipOptimizationObjective::WipOptimizationObjective(ompl::base::SpaceInformationPtr si,
                                                   std::shared_ptr<GpuVoxelsVictor> victor_model)
    : ob::OptimizationObjective(si)
{
    victor_model_ = victor_model;
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
     
