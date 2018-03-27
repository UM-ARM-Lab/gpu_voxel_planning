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
                                 std::shared_ptr<GpuVoxelsVictor> victor_model)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{
    si_ = si;
    stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);
    victor_model_ = victor_model;
}


VictorValidator::~VictorValidator()
{
    std::cout << "Reset called\n";
}


/*
 *  checks if VICTOR_ACTUAL_MAP is currently in collision with known obstacles
 */
// bool VictorValidator::isCurrentlyValid() const
// {
//     return gvl->getMap(VICTOR_ACTUAL_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(ENV_MAP)->as<voxelmap::ProbVoxelMap>()) == 0;
// }


bool VictorValidator::isValid(const ob::State *state) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    if (!victor_model_->isInJointLimits(values))
    {
        return false;
    }
    // robot::JointValueMap state_joint_values = victor_model_->toRightJointValueMap<const double*>(values);
    robot::JointValueMap state_joint_values = victor_model_->toRightJointValueMap(values);
    return victor_model_->queryFreeConfiguration(state_joint_values);
}



bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                       std::pair< ob::State*, double > & lastValid) const
{
    PROFILE_START(CHECK_MOTION_COMP_CHECK);

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
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
    }

    if (result)
        if (!isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    PROFILE_RECORD(CHECK_MOTION_COMP_CHECK);
    return result;
}


bool VictorValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    PROFILE_START(CHECK_MOTION_SIMPLE_CHECK);
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            // const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;


            PROFILE_START(CHECK_MOTION_SIMPLE_INSERTION);

            if(!isValid(test))
            {
                result = false;
                break;
            }
            PROFILE_RECORD(CHECK_MOTION_SIMPLE_INSERTION);

        }
        si_->freeState(test);

        // PROFILE_START(CHECK_MOTION_SIMPLE_COLLISION_TEST);
        // size_t num_colls_pc = gvl->getMap(VICTOR_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(ENV_MAP)->as<voxelmap::ProbVoxelMap>());
        // PROFILE_RECORD(CHECK_MOTION_SIMPLE_COLLISION_TEST);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    PROFILE_RECORD(CHECK_MOTION_SIMPLE_CHECK);
    return result;
}


void VictorValidator::insertStartAndGoal(const ob::ScopedState<> &start, const ob::ScopedState<> &goal) const
{
    
}


