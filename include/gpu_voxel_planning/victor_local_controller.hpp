#ifndef VICTOR_LOCAL_CONTROLLER_HPP
#define VICTOR_LOCAL_CONTROLLER_HPP

#include "victor_validator.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "lazyrrt_fullpath.h"
#include "cost_rrtconnect.h"


namespace gpu_voxels_planner
{


    // typedef ompl::base::ScopedState<> Goals;
    typedef std::shared_ptr<ompl::geometric::PathGeometric> Ompl_Path;
        
    class VictorLocalController
    {
    public:
        VictorLocalController(GpuVoxelsVictor* victor_model);

        Path maxExpectedChsIG(std::vector<double> start_values,
                              double max_motion,
                              int num_samples);


    protected:
        std::shared_ptr<ompl::base::RealVectorStateSpace> space;
        std::shared_ptr<ompl::base::SpaceInformation> si_;
        GpuVoxelsVictor* victor_model_;
    };
}


#endif
