

#include "victor_controller.hpp"
#include "hardcoded_params.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;



Controller::Controller(std::shared_ptr<ob::SpaceInformation> si_inp,
                       std::shared_ptr<ob::RealVectorStateSpace> space_inp,
                       std::shared_ptr<og::PathValidator> vppc_inp,
                       double cost_limit_inp)
{
    si_ = si_inp;
    space = space_inp;
    vppc = vppc_inp;
    cost_limit = cost_limit_inp;
}


Maybe::Maybe<ob::PathPtr> Controller::localControl(ob::ScopedState<> start, Goals goals)
{
    std::cout << "Using local controller\n";
    std::cout << "Cost threshold: " << cost_limit << "\n";
    if(vppc.get() == nullptr)
    {
        std::cout << "Cannot run local controller without allocated rplanner\n";
        return Maybe::Maybe<ob::PathPtr>();
    }
    
    int num_samples = NUM_CONTROLLER_SAMPLES;
    int i=0;

    double start_d_to_goal = goals.distance(start);


    double max_motion = space->getLongestValidSegmentLength() * 5;

    ob::State* test(si_->allocState());
    ob::State* best(si_->allocState());
    double best_ev = 0;
    bool found_ok = false;

    ob::StateSamplerPtr sampler_ = si_->allocStateSampler();
    vppc->setProbabilityThreshold(std::numeric_limits<double>::max());
    double best_cost = std::numeric_limits<double>::max();

    double goal_sampled = false;

    
    // while(i < num_samples)
    for(i=0; i<num_samples; i++)
    {
        if(i==0 && goals.distance(start) <= max_motion*1.5)
        {
            si_->copyState(test, goals.get());
            goal_sampled = true;
        }
        else
        {
            sampler_->sampleUniform(test);

            //uniformly sample motion directions, not states
            double *testv = test->as<ob::RealVectorStateSpace::StateType>()->values;
            const double *startv = start->as<ob::RealVectorStateSpace::StateType>()->values;
            for(int i=0; i<7; i++)
            {
                testv[i] += startv[i];
            }
        }
        
        double motion_dist = si_->distance(start.get(), test);

        if(motion_dist > max_motion)
        {
            space->interpolate(start.get(), test, max_motion / motion_dist, test);
        }

        double progress_d = start_d_to_goal - goals.distance(test);
        
        if(progress_d <= 0 || progress_d <= best_ev)
        {
            continue;
        }


        std::vector<ob::State*> dpath;
        dpath.push_back(start.get());
        dpath.push_back(test);

        size_t col_index;
        // vppc->do_delay = true;
        double cost = vppc->getPathCost(dpath, col_index);

        if(cost > cost_limit)
        {
            continue;
        }

        // double ev = (1.0 - pcol) * (progress_d);
        // double ev = (1.0 - pcol);
        if(cost < best_cost)
        {
            si_->copyState(best, test);
            best_cost = cost;
            found_ok = true;
            // best_pcol = pcol;
        }
    
    }

    si_->freeState(test);
    
    if(!found_ok)
    {
        return Maybe::Maybe<ob::PathPtr>();
    }


    auto path(std::make_shared<og::PathGeometric>(si_));
    path->append(start.get());
    path->append(best);

    return Maybe::Maybe<ob::PathPtr>(path);

    
}
