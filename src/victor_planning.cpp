#include "victor_planning.hpp"


#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include "victor_controller.hpp"


#include <signal.h>
#include <iostream>

#include "hardcoded_params.h"
#include "custom_rrtconnect.h"

#include "cost_simplifier.h"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/geometric/PathSimplifier.h>


#include <stdlib.h>

#include <memory>



namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;



const std::string FULL_PLANNING = "Full Planning";
const std::string PLANNING = "unsmoothed planning";
const std::string POST_PROCESSING = "post processing";
const std::string SMOOTHING = "smoothing";
const std::string VISUALIZE_SOLUTION = "visualize solution";

const std::string INSERT_VIZ_SOLUTION = "insert into viz solution";

const std::string ISVALID_INSERTION = "isValid insertion";
const std::string QUERY_INSERTION = "query insertion";
const std::string ISVALID_COLLISION_TEST = "isValid collision test";
const std::string CHECK_MOTION_SIMPLE_INSERTION = "checkMotion (simple) insertion";
const std::string CHECK_MOTION_SIMPLE_COLLISION_TEST = "checkMotion (simple) collision test";
const std::string CHECK_MOTION_SIMPLE_CHECK = "checkMotion (simple) full check";
const std::string CHECK_MOTION_COMP_CHECK = "checkMotion (complicated) full check";

const std::string STATE_COST = "stateCost";
const std::string MOTION_COST = "motionCost";




VictorPlanner::VictorPlanner(GpuVoxelsVictor* victor_model)
{
    space = std::make_shared<ob::RealVectorStateSpace>();

    double torad=3.1415/180;
    // ob::RealVectorBounds bounds(7);
    // bounds.setLow(-3.14159265);
    // bounds.setHigh(3.14159265);
    // space->setBounds(bounds);


    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-170*torad, 170*torad);
    space->addDimension(-120*torad, 120*torad);
    space->addDimension(-175*torad, 175*torad);

    
    si_ = std::make_shared<ob::SpaceInformation>(space);
    // vv_ptr = std::shared_ptr<VictorValidator>(std::make_shared<VictorValidator>(si_, victor_model));
    simp_ = std::make_shared<og::PathSimplifier>(si_);
    victor_model_ = victor_model;
    use_chs = true;

}

void VictorPlanner::setupSpaceInformation()
{
    std::cout << "using original setupSpaceInformation\n";
    si_->setStateValidityChecker(vv_ptr->getptr());
    si_->setMotionValidator(vv_ptr->getptr());
    si_->setup();
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::planPath(ob::ScopedState<> start, Goals goals)
{
    preparePlanner(start, goals);
    ob::PlannerStatus solved = planner_->solve(10);

    while(solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "Approximate solution, replanning\n";
        solved = planner_->solve(3*10);
    }
    
    std::cout << solved << "\n";
    ob::PathPtr path;    
    //If a solution has been found, we simplify and display it.
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        path = pdef_->getSolutionPath();
        // std::cout << "Found solution. Simplifying..." << std::endl;
        // print the path to screen
        // path->print(std::cout);

        PROFILE_START(SMOOTHING);
        simp_->simplifyMax(*(path->as<og::PathGeometric>()));
        PROFILE_RECORD(SMOOTHING);

        // std::cout << "Simplified solution:" << std::endl;
        // print the path to screen
        // path->print(std::cout);
        PROFILE_START(VISUALIZE_SOLUTION);
        
        // victor_model->visualizeSolution(path);
        
        PROFILE_RECORD(VISUALIZE_SOLUTION);

        std::cout << "Calling post planning actions\n";
        post_planning_actions(path);

    }else{
        std::cout << "No solution could be found" << std::endl;
        return Maybe::Maybe<ob::PathPtr>();
    }
    
    PROFILE_RECORD(POST_PROCESSING);
    PROFILE_RECORD(FULL_PLANNING);

    std::vector<std::string> summary_names = {
        FULL_PLANNING,
        "~~~~~~~~~~~~",
        PLANNING,
        POST_PROCESSING,
        "~~~~~~~~~~~~",
        SMOOTHING,
        VISUALIZE_SOLUTION,
        INSERT_VIZ_SOLUTION,
        "~~~~~~~~~~~~",
        ISVALID_INSERTION,
        ISVALID_COLLISION_TEST,
        "~~~~~~~~~~~~",
        CHECK_MOTION_SIMPLE_CHECK,
        CHECK_MOTION_SIMPLE_INSERTION,
        CHECK_MOTION_SIMPLE_COLLISION_TEST,
        "~~~~~~~~~~~~",
        CHECK_MOTION_COMP_CHECK,
        "~~~~~~~~~",
        STATE_COST,
        MOTION_COST};
    
        
    PROFILE_PRINT_SUMMARY_FOR_GROUP(summary_names);

    return Maybe::Maybe<ob::PathPtr>(path);
}


Path VictorPlanner::omplPathToDoublePath(og::PathGeometric* ompl_path)
{
    Path d_path;

    ob::StateSpace *stateSpace = si_->getStateSpace().get();
    ob::State *state = si_->allocState();
    for(size_t step = 0; step < ompl_path->getStateCount() - 1; step++)
    {
        ob::State *s1 = ompl_path->getState(step);
        ob::State *s2 = ompl_path->getState(step + 1);
        int nd = stateSpace->validSegmentCount(s1, s2);
        
        for(int j = 0; j<nd; j++)
        {
            stateSpace->interpolate(s1, s2, (double)j / (double)nd, state);
            const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            std::vector<double> d_state;
            for(int i=0; i<7; i++)
            {
                d_state.push_back(values[i]);
            }

            assert(d_state.size() == 7);
            d_path.push_back(d_state);
        }
    }
    ob::State *last = ompl_path->getState(ompl_path->getStateCount()-1);
    const double *values = last->as<ob::RealVectorStateSpace::StateType>()->values;
    std::vector<double> d_state;
    for(int i=0; i<7; i++)
    {
        d_state.push_back(values[i]);
    }
    d_path.push_back(d_state);

    si_->freeState(state);
    
    return d_path;
}


Maybe::Maybe<Path> VictorPlanner::planPathConfig(VictorConfig start, VictorConfig goals)
{
    return planPathDouble(victor_model_->toValues(start), victor_model_->toValues(goals));
}

Maybe::Maybe<Path> VictorPlanner::planPathDouble(std::vector<double> start, std::vector<double> goal)
{
    ob::ScopedState<> start_ss(space);
    ob::ScopedState<> goal_ss(space);
    std::cout << "start: ";
    start_ss = toScopedState(start);

    goal_ss = toScopedState(goal);
    

    
    Maybe::Maybe<ob::PathPtr> ompl_path = planPath(start_ss, goal_ss);
    if(!ompl_path.Valid())
    {
        return Maybe::Maybe<Path>();
    }
    return Maybe::Maybe<Path>(omplPathToDoublePath(ompl_path.Get()->as<og::PathGeometric>()));
}

ob::ScopedState<> VictorPlanner::toScopedState(std::vector<double> ds)
{
    ob::ScopedState<> s(space);
    for(size_t i=0; i<ds.size(); i++)
    {
        s[i] = ds[i];
    }
    return s;
}

Maybe::Maybe<Path> VictorPlanner::localControlConfig(VictorConfig start, VictorConfig goal)
{
    return localControlDouble(victor_model_->toValues(start), victor_model_->toValues(goal));
}

Maybe::Maybe<Path> VictorPlanner::localControlDouble(std::vector<double> start, std::vector<double> goal)
{
    Maybe::Maybe<ob::PathPtr> ompl_path = localControl(toScopedState(start), toScopedState(goal));
    if(!ompl_path.Valid())
    {
        return Maybe::Maybe<Path>();
    }
    return Maybe::Maybe<Path>(omplPathToDoublePath(ompl_path.Get()->as<og::PathGeometric>()));
}



Maybe::Maybe<ob::PathPtr> VictorPlanner::localControl(ob::ScopedState<> start, Goals goals)
{
    double cost_limit = 0.9;
    if(use_chs)
    {
        cost_limit = 0.9;
    }
    else //UCG
    {
        cost_limit = VOX_CONTROLLER_THRESHOLD;
    }
    
    Controller controller(si_, space, vppc, cost_limit);
    return controller.localControl(start, goals);
}





/************************************
 **        Victor LBKPICE          **
 ***********************************/

VictorLBKPiece::VictorLBKPiece(GpuVoxelsVictor* victor_model)
    : VictorPlanner(victor_model)
{
    initializePlanner();
}

void VictorLBKPiece::initializePlanner()
{
    vv_ptr = std::make_shared<VictorConservativeValidator>(si_, victor_model_);
    setupSpaceInformation();



    planner_ = std::make_shared<og::LBKPIECE1>(si_);
    planner_->setup();
}

void VictorLBKPiece::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);

    planner_->setProblemDefinition(pdef_);

}





/***********************************************
 **               Victor LazyRRTF                **
 ***********************************************/

VictorLazyRRTF::VictorLazyRRTF(GpuVoxelsVictor* victor_model)
    : VictorPlanner(victor_model)
{
    initializePlanner();
}

void VictorLazyRRTF::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    std::shared_ptr<og::LazyRRTF> lrrtf = std::make_shared<og::LazyRRTF>(si_);
    pv_ = std::make_shared<VictorPathProbCol>(si_, victor_model_);
    lrrtf->setPathValidator(pv_);
    planner_ = lrrtf;
    planner_->setup();
    threshold = 1.0;
    
    vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
    si_->setStateValidityChecker(vv_ptr->getptr());
}

Maybe::Maybe<ob::PathPtr> VictorLazyRRTF::planPath(ompl::base::ScopedState<> start,
                                                   Goals goals)
{
    std::cout << "Using lazyRRTF planner\n";
    preparePlanner(start, goals);
    ob::PathPtr path;
    int planning_time = PLANNING_TIMEOUT;

    threshold = 1.0;
    pv_->setProbabilityThreshold(threshold);
    ob::PlannerStatus solved = planner_->solve(planning_time);
    

    while(solved)
    {
        threshold -= 0.1;
        if(threshold <= 0.0){
            break;
        }

        std::cout << "threshold " << threshold << "\n";
        pv_->setProbabilityThreshold(threshold);
        solved = planner_->solve(planning_time);
        
    }

    
    if (threshold == 1.0)
    {
        std::cout << "No solution could be found" << std::endl;
        return Maybe::Maybe<ob::PathPtr>();
    }

    path = pdef_->getSolutionPath();

    return Maybe::Maybe<ob::PathPtr>(path);
}


void VictorLazyRRTF::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);
    planner_->setProblemDefinition(pdef_);

}







/************************************
 **          Victor PRM            **
 ***********************************/

VictorPRM::VictorPRM(GpuVoxelsVictor* victor_model)
    : VictorPlanner(victor_model)
{
    initializePlanner();
}

void VictorPRM::initializePlanner()
{
    vv_ptr = std::make_shared<VictorConservativeValidator>(si_, victor_model_);
    setupSpaceInformation();



    planner_ = std::make_shared<og::PRM>(si_);
    planner_->setup();
}

void VictorPRM::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);

    planner_->setProblemDefinition(pdef_);

}








/************************************
 **       Victor RRTConnect        **
 ***********************************/

VictorRRTConnect::VictorRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorPlanner(victor_model)
{
    initializePlanner();
}

void VictorRRTConnect::initializePlanner()
{
    vv_ptr = std::make_shared<VictorConservativeValidator>(si_, victor_model_);
    setupSpaceInformation();



    planner_ = std::make_shared<og::RRTConnect>(si_);
    planner_->setup();
}


void VictorRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);

    planner_->setProblemDefinition(pdef_);

}






/***************************************
 **    Victor ThresholdRRTConnect     **
 ***************************************/

VictorThresholdRRTConnect::VictorThresholdRRTConnect(GpuVoxelsVictor* victor_model, bool chs)
    : VictorPlanner(victor_model)
{
    use_chs = chs;
    initializePlanner();
}

void VictorThresholdRRTConnect::initializePlanner()
{
    vv_ptr = std::make_shared<VictorStateThresholdValidator>(si_, victor_model_, use_chs);
    setupSpaceInformation();



    auto crrt(std::make_shared<og::cRRTConnect>(si_));
    crrt->setRange(0.1);
    planner_ = crrt;
    planner_->setup();
    if(use_chs)
    {
        vppc = std::make_shared<VictorPathProbCol>(si_, victor_model_);
    }
    else
    {
        vppc = std::make_shared<VictorPathVox>(si_, victor_model_);
    }
}


Maybe::Maybe<ob::PathPtr> VictorThresholdRRTConnect::planPath(ompl::base::ScopedState<> start,
                                                              Goals goals)
{

    std::cout << "Using threshold planner\n";
    preparePlanner(start, goals);

    ob::PathPtr path;
    double planning_time = PLANNING_TIMEOUT;
    double eps = 0.0001;

    double threshold = std::numeric_limits<double>::max();
    
    ob::PlannerStatus solved(true, false);
    bool anySolution = false;
    VictorStateThresholdValidator* vv_thresh = dynamic_cast<VictorStateThresholdValidator*>(vv_ptr.get());
    vv_thresh->setCostThreshold(threshold);
    
    arc_utilities::Stopwatch stopwatch;
    double time_left;
    
    while((time_left = (planning_time - stopwatch())) > 0)
    {
        
        // threshold -= 0.01;
        // if(threshold <= 0.0){
        //     std::cout << "0 cost path found. Exiting early\n";
        //     break;
        // }

        std::cout << "threshold " << threshold << "\n";
        
        solved = planner_->solve(time_left);

        if(solved)
        {
            anySolution = true;
            path = pdef_->getSolutionPath();
            threshold = vv_thresh->getPathCost(path->as<og::PathGeometric>()) - eps;
            std::cout << "Max cost on path: " << threshold + eps << "\n";
            vv_thresh->setCostThreshold(threshold);
            preparePlanner(start, goals);
            if(!vv_thresh->isValid(start.get()) || !vv_thresh->isValid(goals.get()))
            {
                std::cout << "Found best path for given start/goal with threshold " << (threshold + eps) <<"\n";
                vv_thresh->setCostThreshold(threshold + 2*eps);
                break;
            }
        }
    }
    std::cout << "Planning finished\n";

    if (!anySolution)
    {
        std::cout << "No solution could be found" << std::endl;

        return Maybe::Maybe<ob::PathPtr>();
    }
    
    (path->as<og::PathGeometric>())->interpolate();
    std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states before smoothing\n";
    simp_->shortcutPath(*(path->as<og::PathGeometric>()), 100, 30);
    // std::cout << "done simplifying\n";
    std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states after smoothing\n";
    

    return Maybe::Maybe<ob::PathPtr>(path);
}


void VictorThresholdRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);
    planner_->setProblemDefinition(pdef_);
}


























/***************************************
 **    Victor MotionCostRRTConnect    **
 ***************************************/

VictorMotionCostRRTConnect::VictorMotionCostRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorPlanner(victor_model)
{
}


void VictorMotionCostRRTConnect::smooth(ob::PathPtr &path)
{
    // rplanner_->pv_->do_delay = true;
    
    // rplanner_->pv_->setProbabilityThreshold(std::numeric_limits<double>::max());
    // rplanner_->pv_->setProbabilityThreshold(unsmoothed_path_prob);

    // double unsmoothed_path_prob = rplanner_->pv_->getPathCost(path->as<og::PathGeometric>()->getStates(), col_index);
    // double unsmoothed_path_prob = best_cost;
    // size_t col_index;
    //Note, this could be higher due to densification.
    

    std::cout << "pv thresh after planning: " << rplanner_->pv_->threshold << "\n";


    bool tmp =    rplanner_->pv_->do_delay;
    rplanner_->pv_->do_delay = false;

    og::CostSimplifier cost_simp(si_, rplanner_->pv_.get());
    cost_simp.shortcutPath(*(path->as<og::PathGeometric>()), SMOOTHING_ITERATIONS);

    rplanner_->pv_->do_delay = tmp;


    // double smoothed_path_prob = rplanner_->pv_->getPathCost(path->as<og::PathGeometric>()->getStates(), col_index);

    // std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states after smoothing with cost " << smoothed_path_prob << "\n";

    // assert(smoothed_path_prob <= unsmoothed_path_prob);

    rplanner_->pv_->do_delay = false;

}

Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planAnytime(ob::ScopedState<> start,
                                                                  Goals goals)
{
    preparePlanner(start, goals);
    ob::PathPtr path;
    double planning_time = PLANNING_TIMEOUT;
    double eps = 0.0001;
    double threshold = std::numeric_limits<double>::max();

    bool any_solution = false;

    rplanner_->setProbabilityThreshold(threshold);
    double best_cost = threshold;

    arc_utilities::Stopwatch stopwatch;
    double time_left;

    std::string profile_optim_name = name + " optimal value";
    PROFILE_START(profile_optim_name);
    PROFILE_RECORD_DOUBLE(profile_optim_name, -1);

    
    while((time_left = (planning_time - stopwatch())) > 0)
    {
        std::cout << "threshold " << threshold << "\n";
        
        ob::PlannerStatus solved = planner_->solve(time_left);

        if(solved)
        {
            any_solution = true;

            ob::PathPtr ptmp = pdef_->getSolutionPath();
            double path_cost = rplanner_->path_cost;
            
            std::cout << "Path cost: " << path_cost << "\n";
            if(path_cost < threshold)
            {
                PROFILE_RECORD_DOUBLE(profile_optim_name, path_cost);
                best_cost = path_cost;
                threshold = (path_cost - eps);
                path = ptmp;
                rplanner_->setProbabilityThreshold(threshold);
                // std::cout << "pv thresh: " << rplanner_->pv_->threshold << "\n";
            }
            else{
                std::cout << "This should not happen with planner modifications\n";
                assert(false);
            }

            if(threshold <= 0.0)
            {
                std::cout << "Early termination because 0 cost path found!\n";
                break;
            }
            
            preparePlanner(start, goals);
        }
    }
    std::cout << "Planning finished\n";

    if (!any_solution)
    {
        std::cout << "No solution could be found" << std::endl;

        return Maybe::Maybe<ob::PathPtr>();
    }
    rplanner_->pv_->setProbabilityThreshold(best_cost);
    (path->as<og::PathGeometric>())->interpolate();
    return Maybe::Maybe<ob::PathPtr>(path);
}



Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planUp(ob::ScopedState<> start,
                                                             Goals goals)
{
    std::cout << "Planning up!\n";
    preparePlanner(start, goals);
    ob::PathPtr path;
    double planning_time = PLANNING_TIMEOUT;
    double eps = 0.0001;
    double initial_thresh = 0.3;
    double threshold = initial_thresh;

    bool any_solution = false;

    rplanner_->setProbabilityThreshold(threshold);
    double best_cost = threshold;

    arc_utilities::Stopwatch stopwatch;
    double time_left;

    std::string profile_optim_name = name + " optimal value";
    PROFILE_START(profile_optim_name);
    // PROFILE_RECORD_DOUBLE(profile_optim_name, -1);
    
    while((time_left = (planning_time - stopwatch())) > 0)
    {
        std::cout << "threshold " << threshold << "\n";

        double iter_plan_time = any_solution ? time_left : time_left / 4;
        
        ob::PlannerStatus solved = planner_->solve(iter_plan_time);

        if(solved)
        {
            any_solution = true;

            ob::PathPtr ptmp = pdef_->getSolutionPath();
            double path_cost = rplanner_->path_cost;
            
            std::cout << "Path found with cost: " << path_cost << "\n";
            if(path_cost < threshold)
            {
                PROFILE_RECORD_DOUBLE(profile_optim_name, path_cost);
                best_cost = path_cost;
                threshold = (path_cost - eps);
                path = ptmp;
                rplanner_->setProbabilityThreshold(threshold);
                // std::cout << "pv thresh: " << rplanner_->pv_->threshold << "\n";
            }
            else{
                std::cout << "Shouldn't happend with this rrtconnect\n";
                assert(false);
            }

            if(threshold <= 0.0)
            {
                std::cout << "Early termination because 0 cost path found!\n";
                break;
            }
            
            preparePlanner(start, goals);
        }
        else if(!any_solution)
        {

            threshold = (stopwatch()/planning_time) * cost_upper_bound + initial_thresh;
            rplanner_->setProbabilityThreshold(threshold);
            std::cout << "No solution found, raising threshold to " << threshold << "\n";
        }
        
    }
    std::cout << "Planning finished\n";

    if (!any_solution)
    {
        std::cout << "No solution could be found" << std::endl;

        return Maybe::Maybe<ob::PathPtr>();
    }
    rplanner_->pv_->setProbabilityThreshold(best_cost);
    (path->as<og::PathGeometric>())->interpolate();
    return Maybe::Maybe<ob::PathPtr>(path);
}




Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planPath(ompl::base::ScopedState<> start,
                                                              Goals goals)
{

    Maybe::Maybe<ob::PathPtr> path;
    if(use_anytime_planner)
        path = planAnytime(start, goals);
    else
        path = planUp(start, goals);

    if(!path.Valid())
    {
        return path;
    }


    std::cout << "Path has " << path.Get()->as<og::PathGeometric>()->getStates().size() << " states before smoothing with cost " << rplanner_->path_cost << ",... ";

    smooth(path.Get());
    std::cout << " and " << path.Get()->as<og::PathGeometric>()->getStates().size() << " states after smoothing\n";
    
    return Maybe::Maybe<ob::PathPtr>(path);
}


void VictorMotionCostRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goals);
    planner_->setProblemDefinition(pdef_);
}









/***************************************
 **    Victor VoxCostRRTConnect    **
 ***************************************/
VictorVoxCostRRTConnect::VictorVoxCostRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorMotionCostRRTConnect(victor_model)
{
    std::cout << "Using Voxel Cost planner\n";
    initializePlanner();
    cost_upper_bound = 400;
    use_chs = false;
}


void VictorVoxCostRRTConnect::initializePlanner()
{
    vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
    setupSpaceInformation();

    std::shared_ptr<og::CostRRTConnect> mcrrt = std::make_shared<og::CostRRTConnect>(si_);
    vppc = std::make_shared<VictorPathVox>(si_, victor_model_);
    std::shared_ptr<VictorPathVox> vppc_vox = std::make_shared<VictorPathVox>(si_, victor_model_);
    mcrrt->setPathValidator(vppc_vox);
    planner_ = mcrrt;
    rplanner_ = planner_->as<og::CostRRTConnect>();
    planner_->setup();
    // std::cout << "Planner max extend dist: " << mcrrt->getRange() << "\n";
}







/****************************************
 **    Victor ProbColCostRRTConnect    **
 ****************************************/
VictorProbColCostRRTConnect::VictorProbColCostRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorMotionCostRRTConnect(victor_model)
{
    std::cout << "Using Probabiltiy of Collision planner\n";
    initializePlanner();
    cost_upper_bound = 1.0;
}


void VictorProbColCostRRTConnect::initializePlanner()
{
    vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
    setupSpaceInformation();

    std::shared_ptr<og::CostRRTConnect> mcrrt = std::make_shared<og::ProbColRRTConnect>(si_);
    vppc = std::make_shared<VictorPathProbCol>(si_, victor_model_);

    mcrrt->setPathValidator(vppc);
    planner_ = mcrrt;
    rplanner_ = planner_->as<og::CostRRTConnect>();
    planner_->setup();
    // std::cout << "Planner max extend dist: " << mcrrt->getRange() << "\n";
}
