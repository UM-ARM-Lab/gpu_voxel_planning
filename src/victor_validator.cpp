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
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    VictorConfig config = victor_model_->toVictorConfig(values);
    
    return victor_model_->countTotalNumCollisionsForConfig(config) == 0;
}






/***********************************************
 **            PATH VALIDATOR                 **
 ***********************************************/
VictorPathValidator::VictorPathValidator(const ob::SpaceInformationPtr &si,
                                         GpuVoxelsVictor* victor_model) :
    PathValidator(si)
{
    victor_model_ = victor_model;
}

void VictorPathValidator::setProbabilityThreshold(double th)
{
    threshold = th;
}

bool VictorPathValidator::checkPath(const std::vector<ompl::base::State*> path,
                                    size_t &collision_index)
{
    // std::cout << "Checking path...";

    
    ompl::base::StateSpace *stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_model_->resetQuery();
    ob::State *test = si_->allocState();

    double prob_col = 0.0;
        
    for(collision_index = 0; collision_index < (path.size() - 1); collision_index ++)
    {
        const ob::State *s1 = path[collision_index];
        const ob::State *s2 = path[collision_index + 1];
        int nd = stateSpace_->validSegmentCount(s1, s2);

        for(int j = 0; j < nd; j++)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;
            victor_model_->addQueryState(victor_model_->toVictorConfig(values));
        }
        std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();
        std::vector<size_t> seen_sizes = victor_model_->seenSizes();
        std::vector<double> p_no_collision;
        p_no_collision.resize(seen_col_voxels.size());
        double p_no_col_seen = 1.0;

        for(size_t i=0; i < seen_sizes.size(); i++)
        {
            p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
            assert(p_no_collision[i] <= 1.0);
            p_no_col_seen *= p_no_collision[i];
        }

        victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
        // std::cout << "p_no_col_seen: " << p_no_col_seen << "\n";
        // int unused;
        // std::cout << "Waiting for user input to start...\n";
        // std::cin >> unused;


        size_t path_size = victor_model_->countIntersect(FULL_MAP, VICTOR_QUERY_MAP);
        size_t total_size = victor_model_->countIntersect(FULL_MAP, FULL_MAP);
        size_t known_free_size = victor_model_->countIntersect(VICTOR_SWEPT_VOLUME_MAP, VICTOR_QUERY_MAP);

        double num_occupied = (double)(path_size - known_free_size);
        double frac_occupied = num_occupied / (double) total_size;
        
        double p_no_col_unseen = std::pow(1.0 - frac_occupied, 0);

        prob_col = 1.0 - p_no_col_seen * p_no_col_unseen;

        if (prob_col > 1.0)
        {
            std::cout << "Prob_col " << prob_col;
            std::cout << ", p_no_col_seen " << p_no_col_seen;
            std::cout << ", p_no_col_unseen " << p_no_col_unseen;
            // assert (prob_col <= 1.0);
        }

        
        if(prob_col > threshold)
        {
            break;
        }
    }
    si_->freeState(test);
    

    // std::cout << "Finished\n";
    // std::cout << "pathsize " << path.size();
    // std::cout << " col index " << collision_index << "\n";
    // std::cout << "Threshold " << threshold << "\n";
    return prob_col <= threshold;
}




