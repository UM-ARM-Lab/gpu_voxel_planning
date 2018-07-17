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
