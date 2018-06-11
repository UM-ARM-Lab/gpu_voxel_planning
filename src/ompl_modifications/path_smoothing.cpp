#include "path_smoothing.h"
#include <random>

namespace ob = ompl::base;
namespace og = ompl::geometric;



void og::PathSmoother::shortcutPath(og::PathGeometric &path, size_t num_trials)
{
    rng.seed(std::random_device()());

    // std::cout << "path size in smoother before: "<< path.getStates().size() << "\n";
    pv_->setProbabilityThreshold(std::numeric_limits<double>::max());
    size_t col_index;
    bool fast_check = true;
    // std::cout << "calling with fast check " << fast_check << "\n";
    cur_cost = pv_->getPathCost(path.getStates(), col_index, fast_check);
    // std::cout << "Current path cost " << cur_cost << "\n";

    bool debug = false;
    if(debug)
    {
        pv_->do_delay = true;
        std::cout << "Original path with cost " << cur_cost << "\n";
        std::string unused;
        std::getline(std::cin, unused);

    }

    pv_->setProbabilityThreshold(cur_cost + 3*eps);
    
    for(size_t i=0; i<num_trials; i++)
    {
        singleShortcut(path);
    }
    // std::cout << " and after: "<< path.getStates().size() << "\n";   
}

void og::PathSmoother::sampleInd(int &start, int &end, int max_exclusive)
{
    start = 0;
    end = 0;
    if(max_exclusive <= 2)
        return;

    std::uniform_int_distribution<size_t> dist(0, max_exclusive);

    while(start >= end-1)
    {
        int a = dist(rng);
        end = dist(rng);
        start = a<end ? a : end;
        end = a < end ? end : a;
    }
}


/**
 *   Shortcut smoothing where there is not a state validity check,
 *    but instead a motion cost check
 */
void og::PathSmoother::singleShortcut(og::PathGeometric &path)
{

    bool fast_check = true;

    std::vector<ob::State *> &states = path.getStates();

    if(states.size() <= 2)
    {
        return;
    }
    
    int start_ind, end_ind;
    sampleInd(start_ind, end_ind, states.size());

    if(start_ind >= end_ind)
    {
        std::cout << "start: " << start_ind << " end: " << end_ind << " sizes: " << states.size() << "\n";
        assert(start_ind < end_ind);
    }

    
    std::vector<ob::State *> orig_segment(states.begin()+start_ind, states.begin()+end_ind);

    size_t col_index;
    ob::StateSpace *stateSpace = si_->getStateSpace().get();
    
    int nd = stateSpace->validSegmentCount(orig_segment[0], orig_segment.back());
    std::vector<ob::State *> new_segment;

    si_->getMotionStates(orig_segment[0], orig_segment.back(), new_segment, nd, true, true);
    std::vector<ob::State *> post_rework(states.begin() + end_ind, states.end());


    std::vector<ob::State *> new_path;
    new_path = std::vector<ob::State *> (states.begin(), states.begin()+start_ind);
    new_path.insert(new_path.end(), new_segment.begin(), new_segment.end());
    new_path.insert(new_path.end(), post_rework.begin(), post_rework.end());


    bool possibly_good = true;
    if(new_path.size() >= states.size())
    {
        // std::cout << "shortcut is not smaller, exiting early\n";
        possibly_good = false;
        si_->freeStates(new_segment);
        return;
    }

    if((double)new_segment.size()/(double)orig_segment.size() > .8)
    {
        // std::cout << "not sufficiently shorter\n";
        // return;
    }


    
    double new_seg_cost = pv_->getPathCost(new_segment, col_index, fast_check);
    if(possibly_good && new_seg_cost > cur_cost)
    {
        // std::cout << "new segment has cost (" << new_seg_cost<< ") higher than total path, exiting early\n";
        possibly_good = false;
        si_->freeStates(new_segment);
        return;
    }

    
    // double orig_seg_cost = pv_->getPathCost(orig_segment, col_index, fast_check);
    // if(orig_seg_cost < new_seg_cost)
    // {
    //     std::cout << "new segment has cost ( " << new_seg_cost << ") higher than old segment(";
    //     std::cout << orig_seg_cost << "), exiting early\n";
    //     si_->freeStates(new_segment);
    //     return;
    // }




    // std::reverse(std::begin(new_path), std::end(new_path));
    double new_cost = pv_->getPathCost(new_path, col_index, fast_check);
    // std::reverse(std::begin(new_path), std::end(new_path));


    if(possibly_good && new_cost <= cur_cost)
    {
        // std::cout << "smoother path found with cost " << new_cost << "\n";
        cur_cost = new_cost;
        states = new_path;
        si_->freeStates(orig_segment);
        pv_->setProbabilityThreshold(new_cost + eps);
    }
    else{
        // std::cout << "new total path has higher cost: ("<< new_cost<< ") than old path: " << cur_cost << "\n";
        si_->freeStates(new_segment);
    }

    bool debug = false;
    if(debug)
    {
        std::cout << "start_ind: " << start_ind << " end_ind: " << end_ind << "\n";
        std::string unused;
        std::getline(std::cin, unused);
    }

}
