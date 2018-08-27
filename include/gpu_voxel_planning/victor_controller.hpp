#ifndef VICTOR_CONTROLLER_HPP
#define VICTOR_CONTROLLER_HPP

#include "victor_validator.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "cost_rrtconnect.h"





namespace gpu_voxels_planner
{


    typedef ompl::base::ScopedState<> Goals;
        
    class Controller
    {
    public:
        Controller(std::shared_ptr<ompl::base::SpaceInformation> si_inp,
                   std::shared_ptr<ompl::base::RealVectorStateSpace> space_inp,
                   std::shared_ptr<ompl::geometric::PathValidator> vppc_inp,
                   double cost_limit_inp);

        Maybe::Maybe<ompl::base::PathPtr> localControl(ompl::base::ScopedState<> start, Goals goals);
        std::shared_ptr<ompl::base::RealVectorStateSpace> space;
        std::shared_ptr<ompl::geometric::PathValidator> vppc;
        std::shared_ptr<ompl::base::SpaceInformation> si_;
        double cost_limit;
    };
}

#endif
