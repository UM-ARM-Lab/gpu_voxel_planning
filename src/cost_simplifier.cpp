#include "cost_simplifier.h"
#include <random>

namespace ob = ompl::base;
namespace og = ompl::geometric;



void og::CostSimplifier::shortcutPath(og::PathGeometric &path, size_t num_trials)
{
    for(size_t i=0; i<num_trials; i++)
    {
        singleShortcut(path);
    }
}


/**
 *   Shortcut smoothing where there is not a state validity check,
 *    but instead a motion cost check
 */
void og::CostSimplifier::singleShortcut(og::PathGeometric &path)
{

    double orig_threshold = pv_->threshold;
    std::vector<ob::State *> &states = path.getStates();
    
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<size_t> dist(0, states.size());


    int a = dist(rng);
    int end_ind = dist(rng);
    int start_ind = a<end_ind ? a : end_ind;
    end_ind = a < end_ind ? end_ind : a;
    if(a == end_ind)
        return;

    assert(start_ind < end_ind);
    
    std::vector<ob::State *> orig(states.begin()+start_ind, states.begin()+end_ind);

    size_t col_index;
    // std::cout << "attempting path cost smoothing from " << start_ind << " to " << end_ind << "\n";
    double cur_cost = pv_->getPathCost(orig, col_index);

    pv_->setProbabilityThreshold(cur_cost);


    ob::StateSpace *stateSpace = si_->getStateSpace().get();
    
    int nd = stateSpace->validSegmentCount(orig[0], orig.back());
    std::vector<ob::State *> new_attempt;

    si_->getMotionStates(orig[0], orig.back(), new_attempt, nd, true, true);
    // for(size_t i=0; i < new_attempt.size(); i++)
    // {
    //     std::cout << "i: " << i << "\n";
    //     new_attempt[i] = si_->allocState();
    //     stateSpace->interpolate(orig[0], orig.back(), (double)i / (double)nd, new_attempt[i]);   
    // }


    // std::cout << "shortcut attempt\n";
    double new_cost = pv_->getPathCost(new_attempt, col_index);

    pv_->setProbabilityThreshold(orig_threshold);


    if(new_cost <= cur_cost)
    {
        // std::cout << "Sizes: before " << states.size();
        std::vector<ob::State *> post_rework(states.begin() + end_ind, states.end());
        
        states = std::vector<ob::State *> (states.begin(), states.begin()+start_ind);
        // states.insert(states.end(), orig.begin(), orig.end());
        states.insert(states.end(), new_attempt.begin(), new_attempt.end());
        states.insert(states.end(), post_rework.begin(), post_rework.end());

        // std::cout << ", after " << states.size() << "\n";
        
        si_->freeStates(orig);

        // std::cout << "Path smoothing worked! New path\n";
        // pv_->getPathCost(states, col_index);
    }
    else{
        si_->freeStates(new_attempt);
    }

}
