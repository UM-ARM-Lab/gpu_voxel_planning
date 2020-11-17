#include "wip_optimization_objective.hpp"
#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <ompl/base/spaces/SE3StateSpace.h>


const std::string STATE_COST = "stateCost";
const std::string MOTION_COST = "motionCost";

namespace ob = ompl::base;
// namespace og = ompl::geometric;

WipOptimizationObjective::WipOptimizationObjective(ompl::base::SpaceInformationPtr si,
                                                   std::shared_ptr<GpuVoxelsVictor> victor_model)
    : ob::OptimizationObjective(si)
{
    victor_model_ = victor_model;
    si_ = si;
}


ob::Cost WipOptimizationObjective::stateCost(const ob::State *s) const
{

    PROFILE_START(STATE_COST);
    PROFILE_RECORD(STATE_COST);
    return ob::Cost();
}

ob::Cost WipOptimizationObjective::motionCost(const ob::State *s1,
                                              const ob::State *s2) const
{
    PROFILE_START(MOTION_COST);

    ompl::base::StateSpace *stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    int nd = stateSpace_->validSegmentCount(s1, s2);

    victor_model_->resetQuery();

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            // PROFILE_START(CHECK_MOTION_SIMPLE_INSERTION);

            robot::JointValueMap state_joint_values = victor_model_->toRightJointValueMap(values);

            victor_model_->addQueryState(state_joint_values);
            // if(!isValid(test))
            // {
            //     result = false;
            //     break;
            // }
            // PROFILE_RECORD(CHECK_MOTION_SIMPLE_INSERTION);

        }
        si_->freeState(test);
    }


    PROFILE_RECORD(MOTION_COST);

    return ob::Cost(victor_model_->countNumCollisions());
}
     
