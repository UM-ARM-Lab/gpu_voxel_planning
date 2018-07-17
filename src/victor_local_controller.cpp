#include "victor_local_controller.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;



VictorLocalController::VictorLocalController(GpuVoxelsVictor* victor_model)
{
    double torad=3.1415/180;
    space = std::make_shared<ob::RealVectorStateSpace>();
    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-175*torad, 175*torad);

    
    si_ = std::make_shared<ob::SpaceInformation>(space);

    victor_model_ = victor_model;

}


ob::ScopedState<> VictorLocalController::samplePointInRandomDirection(ob::ScopedState<> start)
{
    double max_motion = space->getLongestValidSegmentLength() * 2;
    
    ob::ScopedState<> new_state(space);
    ob::StateSamplerPtr sampler_ = si_->allocStateSampler();
    sampler_->sampleUniform(new_state.get());
    const double *startv = start->as<ob::RealVectorStateSpace::StateType>()->values;
    double *new_vals = new_state->as<ob::RealVectorStateSpace::StateType>()->values;
    for(int i=0; i<7; i++)
    {
        new_vals[i] += startv[i];
    }

    double motion_dist = si_->distance(start.get(), new_state.get());

    if(motion_dist > max_motion)
    {
        space->interpolate(start.get(), new_state.get(), max_motion / motion_dist, new_state.get());
    }

    return new_state;
}
