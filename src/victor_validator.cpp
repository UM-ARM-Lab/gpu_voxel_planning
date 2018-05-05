#include "victor_validator.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include "common_names.hpp"

#include <thrust/extrema.h>
#include <algorithm>
#include "hardcoded_params.h"

#include <thread>
#include <chrono>
#include <limits>

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




/****************************************
 **     Victor Threshold Validator     **
 ***************************************/

VictorStateThresholdValidator::VictorStateThresholdValidator(const ompl::base::SpaceInformationPtr &si,
                                                   GpuVoxelsVictor* victor_model):
    VictorValidator(si, victor_model)
{
    threshold = 1.0;
    use_prob_col = false;
    use_vox = false;
}





double VictorStateThresholdValidator::getCollisionProb(const ob::State *state) const
{
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    VictorConfig config = victor_model_->toVictorConfig(values);

    victor_model_->resetQuery();
    victor_model_->addQueryState(config);

    if(victor_model_->countNumCollisions(KNOWN_OBSTACLES_MAP) > 0)
    {
        return 1.0;
    }


    std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();
    std::vector<size_t> seen_sizes = victor_model_->seenSizes();


    // std::cout << "Reporting on query\n";
    double p_no_collision = 1.0;
    for(size_t i=0; i<seen_sizes.size(); i++)
    {
        // std::cout << "i:\n";
        // std::cout << "seen col voxels: " << seen_col_voxels[i] << "\n";
        // std::cout << "seen sizes: " << seen_sizes[i] << "\n";
        p_no_collision *= (1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i]);
    }
    assert(p_no_collision <= 1.0);
    assert(p_no_collision >= 0.0);
    double p_collision = 1.0 - p_no_collision;
    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    // std::cout << "p_collision: " << p_collision << "\n";
    // int unused;
    // std::cin >> unused;
    // usleep(100000);
    // std::cout << "threshold: " << threshold << "\n";
    return p_collision;
}

double VictorStateThresholdValidator::getColVoxelIntersects(const ob::State *state) const
{
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    VictorConfig config = victor_model_->toVictorConfig(values);

    victor_model_->resetQuery();
    victor_model_->addQueryState(config);
    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    if(victor_model_->countNumCollisions(KNOWN_OBSTACLES_MAP) > 0)
    {
        return std::numeric_limits<double>::max();
    }

    return victor_model_->countIntersect(VICTOR_QUERY_MAP, COMBINED_COLSETS_MAP);
}


bool VictorStateThresholdValidator::isValid(const ob::State *state) const
{
    if(!VictorValidator::isValid(state))
    {
        return false;
    }
    // std::cout << "\n\n\n\n!!!!!!!!!!!11 Validity Check Called !!!!+!!!!\n";
    bool valid;
    // std::cout << "validating for threshold " << threshold << "\n";
    if(use_prob_col)
    {
        // std::cout << "Using prob col validator";
        // std::cout << "col prob " << getCollisionProb(state) << "\n";
        valid = getCollisionProb(state) < threshold;
    }
    else if(use_vox)
    {
        // std::cout << "Using vox validator\n";
        valid = getColVoxelIntersects(state) < threshold;
    }
    else
    {
        std::cout << "Threshold isValid called but not use_vox nor use_prob_col indicated\n";
        assert(false);
    }
    // std::cout << "Validity Check Called " << valid << "\n";
    return valid;
}


double VictorStateThresholdValidator::getMotionColProb(ob::State *s1, ob::State *s2) const
{
    ob::StateSpace *stateSpace = si_->getStateSpace().get();
    ob::State *state = si_->allocState();
    int nd = stateSpace->validSegmentCount(s1, s2);
    double max_col_prob = 0.0;

    for(int i=0; i<nd; i++)
    {
        stateSpace->interpolate(s1, s2, (double)i / (double)nd, state);
        max_col_prob = std::max<double>(max_col_prob, getCollisionProb(state));
    }
    return max_col_prob;
}

double VictorStateThresholdValidator::getPathMaxColProb(og::PathGeometric *path) const
{
    std::vector<ob::State*> states = path->getStates();
    double max_col_prob = 0.0;
    // std::cout << "Validating " << states.size() << " states\n";
    // int unused;
    // std::cin >> unused;
    // for(auto state: states)
    for(size_t i = 1; i < states.size(); i++)
    {
        // auto state = states[i];
        double pcol = getMotionColProb(states[i-1], states[i]);
        // std::cout << "PathMax pcol: " << pcol << "\n";
        if(pcol > max_col_prob)
        {
            max_col_prob = pcol;
        }
        // std::string unused;
        // std::getline(std::cin, unused);

        // usleep(100000);
    }
    return max_col_prob;
}

double VictorStateThresholdValidator::getPathMaxVox(og::PathGeometric *path) const
{
    std::vector<ob::State*> states = path->getStates();
    double max_cost = 0.0;
    for(size_t i = 0; i < states.size(); i++)
    {
        double scost = getColVoxelIntersects(states[i]);
        max_cost = std::max(max_cost, scost);

    }
    return max_cost;

}

double VictorStateThresholdValidator::getPathCost(og::PathGeometric *path) const
{
    if(use_prob_col)
    {
        return getPathMaxColProb(path);
    }
    else if(use_vox)
    {
        return getPathMaxVox(path);
    }
    else{
        std::cout << "Attempted to get path cost without useprobcol or use_vox\n";
        assert(false);
    }
    
}





















/***********************************************
 **        ProbCol PATH VALIDATOR             **
 ***********************************************/
VictorPathProbCol::VictorPathProbCol(const ob::SpaceInformationPtr &si,
                                         GpuVoxelsVictor* victor_model) :
    PathValidator(si)
{
    victor_model_ = victor_model;
}



double VictorPathProbCol::getPathCost(const std::vector<ob::State*> path,
                                      size_t &collision_index, bool fast)
{
    std::vector<double> costs;
    double c = getPathCost(path, costs, fast);
    collision_index = costs.size();
    return c;
}


/*
 *  Returns the probability of collision for a path.
 *  Early termination if prob of collision is above threshold
 */
double VictorPathProbCol::getPathCost(const std::vector<ob::State*> path,
                                      std::vector<double> &costs, bool fast)
{
    PROFILE_START("getProbCost");
    // std::cout << "Validator threshold "<< threshold << "\n";

    ompl::base::StateSpace *stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_model_->resetQuery();
    ob::State *test = si_->allocState();

    double prob_col = 0.0;
    double p_no_col_unseen;
    double p_no_col_seen;
    
    std::vector<size_t> seen_sizes = victor_model_->seenSizes();

    if(do_delay)
    {
        std::cout << "Running getPathCost for path of size " << path.size() << "\n";
    }
    
    for(size_t collision_index = 0; collision_index < (path.size() - 1); collision_index ++)
    {
        const ob::State *s1 = path[collision_index];
        const ob::State *s2 = path[collision_index + 1];
        if(!si_->isValid(s1))
        {
            PROFILE_RECORD("getProbCost");
            prob_col = 1.0;
            break;
        }
        int nd = stateSpace_->validSegmentCount(s1, s2);
        if(nd==2)
        {
            nd = 1; //path already densified
        }

        if(do_delay)
        {
            // std::cout << "nd: " << nd << "\n";
        }

        
        PROFILE_START("prob add query");
        for(int j = 0; j < nd; j++)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;
            victor_model_->addQueryState(victor_model_->toVictorConfig(values));
        }
        PROFILE_RECORD("prob add query");
        if(fast)
        {
            continue;
        }


        if(victor_model_->countNumCollisions(KNOWN_OBSTACLES_MAP) > 0)
        {
            PROFILE_RECORD("getProbCost");
            if(do_delay)
            {
                std::cout << "terminating early, known cost collision\n";
            }
            prob_col = 1.0;
            break;
        }


        PROFILE_START("prob seen collision");
        std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();

        PROFILE_RECORD("prob seen collision")
                    
        std::vector<double> p_no_collision;
        p_no_collision.resize(seen_col_voxels.size());
        p_no_col_seen = 1.0;

        for(size_t i=0; i < seen_sizes.size(); i++)
        {
            assert(seen_sizes[i] > 0);
            p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
            if(p_no_collision[i] > 1.0)
            {
                std::cout << "seen col vox: " << seen_col_voxels[i] << " seen sizes " << seen_sizes[i] << "\n";
                assert(p_no_collision[i] <= 1.0);
            }
            p_no_col_seen *= p_no_collision[i];
        }

        victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);

        PROFILE_START("prob free space collisions");
        size_t path_size = victor_model_->countIntersect(FULL_MAP, VICTOR_QUERY_MAP);
        size_t total_size = victor_model_->countIntersect(FULL_MAP, FULL_MAP);
        size_t known_free_size = victor_model_->countIntersect(VICTOR_SWEPT_VOLUME_MAP, VICTOR_QUERY_MAP);
        PROFILE_RECORD("prob free space collisions");

        double num_occupied = (double)(path_size - known_free_size);
        double frac_occupied = num_occupied / (double) total_size;
        
        p_no_col_unseen = std::pow(1.0 - frac_occupied, UNEXPLORED_BIAS);

        prob_col = 1.0 - p_no_col_seen * p_no_col_unseen;
        
        if (prob_col > 1.0)
        {
            std::cout << "Prob_col " << prob_col;
            std::cout << ", p_no_col_seen " << p_no_col_seen;
            std::cout << ", p_no_col_unseen " << p_no_col_unseen;
            std::cout << "\n";
            assert (prob_col <= 1.0);
        }

        if(do_delay)
        {
            // std::cout << "Prob col: " << prob_col << "\n";
            // std::cout << "prob colsets: " << 1.0-p_no_col_seen;
            // std::cout << "   prob void space: " << 1.0 - p_no_col_unseen << "\n";
            // usleep(30000);
        }
        
        if(prob_col > threshold)
        {
            if(do_delay)
            {
                std::cout << "PROB COL ABOVE THRESHOLD " << threshold << "\n";
            }
            break;
        }
        costs.push_back(prob_col);
    }
    si_->freeState(test);

    if(fast && prob_col < 1.0)
    {
        if(victor_model_->countNumCollisions(KNOWN_OBSTACLES_MAP) > 0)
        {
            PROFILE_RECORD("getProbCost");
            prob_col = 1.0;
        } else
        {
            PROFILE_START("prob seen collision");
            std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();
            std::vector<size_t> seen_sizes = victor_model_->seenSizes();
            PROFILE_RECORD("prob seen collision");
                
            std::vector<double> p_no_collision;
            p_no_collision.resize(seen_col_voxels.size());
            p_no_col_seen = 1.0;

            for(size_t i=0; i < seen_sizes.size(); i++)
            {
                p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
                assert(p_no_collision[i] <= 1.0);
                p_no_col_seen *= p_no_collision[i];
            }

            victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);

            PROFILE_START("prob free space collisions");
            size_t path_size = victor_model_->countIntersect(FULL_MAP, VICTOR_QUERY_MAP);
            size_t total_size = victor_model_->countIntersect(FULL_MAP, FULL_MAP);
            size_t known_free_size = victor_model_->countIntersect(VICTOR_SWEPT_VOLUME_MAP, VICTOR_QUERY_MAP);
            PROFILE_RECORD("prob free space collisions");

            double num_occupied = (double)(path_size - known_free_size);
            double frac_occupied = num_occupied / (double) total_size;
        
            p_no_col_unseen = std::pow(1.0 - frac_occupied, UNEXPLORED_BIAS);

            prob_col = 1.0 - p_no_col_seen * p_no_col_unseen;
        }

    }

    if(do_delay)
    {
        std::cout << "path cost complete, waiting for input to continue\n";
        victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
        std::string unused;
        std::getline(std::cin, unused);
    }
    PROFILE_RECORD("getProbCost");


    return prob_col;
}

bool VictorPathProbCol::checkPath(const std::vector<ob::State*> path,
                                    size_t &collision_index)
{
    return getPathCost(path, collision_index) <= threshold;
}
















/***********************************************
 **        PATH VOXEL COUNT                   **
 ***********************************************/
VictorPathVox::VictorPathVox(const ob::SpaceInformationPtr &si,
                                         GpuVoxelsVictor* victor_model) :
    PathValidator(si)
{
    victor_model_ = victor_model;
}


double VictorPathVox::getPathCost(const std::vector<ob::State*> path,
                                  size_t &collision_index, bool fast)
{
    std::vector<double> costs;
    double c = getPathCost(path, costs, fast);
    collision_index = costs.size();
    return c;
}


/*
 *  Returns the probability of collision for a path.
 *  Early termination if prob of collision is above threshold
 */
double VictorPathVox::getPathCost(const std::vector<ob::State*> path,
                                  std::vector<double> &costs, bool fast)
{
    PROFILE_START("getVoxCost");
    ompl::base::StateSpace *stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_model_->resetQuery();
    ob::State *test = si_->allocState();

    double total_col = 0.0;
        
    for(size_t collision_index = 0; collision_index < (path.size() - 1); collision_index ++)
    {
        total_col = 0;
        const ob::State *s1 = path[collision_index];
        const ob::State *s2 = path[collision_index + 1];
        if(!si_->isValid(s1))
        {
            PROFILE_RECORD("getVoxCost");
            return std::numeric_limits<double>::max();
        }
        int nd = stateSpace_->validSegmentCount(s1, s2);

        PROFILE_START("vox add query");
        for(int j = 0; j < nd; j++)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;
            victor_model_->addQueryState(victor_model_->toVictorConfig(values));
        }
        PROFILE_RECORD("vox add query");


        if(victor_model_->countNumCollisions(KNOWN_OBSTACLES_MAP) > 0)
        {
            PROFILE_RECORD("getVoxCost");
            return std::numeric_limits<double>::max();
        }

        if(!fast)
        {
            PROFILE_START("vox collision");
            total_col = victor_model_->countIntersect(VICTOR_QUERY_MAP, COMBINED_COLSETS_MAP);

            // std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();
            // for(size_t i=0; i < seen_col_voxels.size(); i++)
            // {
            //     total_col += seen_col_voxels[i];
            // }

            victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
            PROFILE_RECORD("vox collision");
        }

        if(do_delay)
        {
            std::cout << "Total col: " << total_col << "\n";
            usleep(30000);
        }

        if(total_col > threshold)
        {
            if(do_delay)
            {
                std::cout << "PROB COL ABOVE THRESHOLD " << threshold << "\n";
            }
            break;

        }
        costs.push_back(total_col);
    }
    si_->freeState(test);

    if(fast)
    {
        total_col=0;
        PROFILE_START("vox collision");
        std::vector<size_t> seen_col_voxels = victor_model_->countSeenCollisionsInQueryForEach();
        PROFILE_RECORD("vox collision");

        for(size_t i=0; i < seen_col_voxels.size(); i++)
        {
            total_col += seen_col_voxels[i];
        }
        victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    }


    if(do_delay)
    {
        std::cout << "path cost complete, waiting for input to continue\n";
        std::string unused;
        std::getline(std::cin, unused);
    }
    PROFILE_RECORD("getVoxCost");
    
    return total_col;
}


bool VictorPathVox::checkPath(const std::vector<ob::State*> path,
                              size_t &collision_index)
{
    return getPathCost(path, collision_index) <= threshold;
}









/******************************
 **     VictorObjective      **
 *****************************/

VictorObjective::VictorObjective(const ompl::base::SpaceInformationPtr &si,
                                 std::shared_ptr<ompl::geometric::PathValidator> pv) :
    ob::OptimizationObjective(si)
{
    pv_ = pv;
}


ob::Cost VictorObjective::motionCost(const ob::State *s1,
                                     const ob::State *s2) const
{
    size_t col_index;
    bool fast=true;
    std::vector<ob::State*> motion;
    ob::State *s1_ = si_->allocState();
    si_->copyState(s1_, s1);
    ob::State *s2_ = si_->allocState();
    si_->copyState(s2_, s2);
    motion.push_back(s1_);
    motion.push_back(s2_);
    double cost = pv_->getPathCost(motion, col_index, fast);

    si_->freeState(s1_);
    si_->freeState(s2_);
    return ob::Cost(cost);
}

ob::Cost VictorObjective::stateCost(const ob::State *state) const
{
    return motionCost(state, state);
}

ob::Cost VictorObjective::combineCosts(ob::Cost c1, ob::Cost c2) const 
{
    if(is_prob_cost)
    {
        return ob::Cost(1 - (1-c1.value())*(1-c2.value()) );
    }
    else{
        return ob::Cost(c1.value() + c2.value());
    }
}


ob::Cost VictorObjective::identityCost() const
{
    if(is_prob_cost)
    {
        return ob::Cost(1.0);
    }
    else
    {
        return ob::Cost(0.0);
    }
}
