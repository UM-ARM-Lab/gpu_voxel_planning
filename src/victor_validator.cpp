#include "victor_validator.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include "common_names.hpp"

#include <thrust/extrema.h>

#include <thread>
#include <chrono>

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels;
namespace bfs = boost::filesystem;


/*
 *
 *  VictorValidator handles the voxelmaps for victor
 *  Currently this just works for his right arm
 *  
 */



VictorValidator::VictorValidator(const ob::SpaceInformationPtr &si,
                                 GpuVoxelsVictor* victor_model)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si),
    si_(si)
{
    // si_ = si;
    stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);
    victor_model_ = victor_model;
}

VictorValidator::~VictorValidator()
{
    std::cout << "Deleting Validator\n";
}


bool VictorValidator::isValid(const ob::State *state) const
{
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    return victor_model_->isInJointLimits(values);
}



bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                       std::pair< ob::State*, double > & lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    ob::State *test = si_->allocState();
    for (int j = 1; j <= nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        // if (!si_->isValid(test))
        if (!isValid(test))
        {
            lastValid.second = (double)(j - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
            break;
        }
    }
    si_->freeState(test);


    valid_ += result;
    invalid_ += !result; 

    return result;
}


bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* temporary storage for the checked state */
    ob::State *test = si_->allocState();
    for (int j = 1; j <= nd; ++j)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        if(!isValid(test))
        {
            result = false;
            break;
        }

    }
    si_->freeState(test);

    valid_ += result;
    invalid_ += !result; 
    return result;
}




/****************************************
 **    Victor Conservative Validator   **
 ***************************************/

VictorConservativeValidator::VictorConservativeValidator(const ompl::base::SpaceInformationPtr &si,
                                                         GpuVoxelsVictor* victor_model):
    VictorValidator(si, victor_model)
{
}


bool VictorConservativeValidator::isValid(const ob::State *state) const
{
    if(!VictorValidator::isValid(state))
    {
        return false;
    }
}
