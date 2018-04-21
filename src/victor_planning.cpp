#include "victor_planning.hpp"


// #define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>


#include <signal.h>
#include <iostream>

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
    space = std::make_shared<ob::RealVectorStateSpace>(7);

    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
    space->setBounds(bounds);
    si_ = std::make_shared<ob::SpaceInformation>(space);
    // vv_ptr = std::shared_ptr<VictorValidator>(std::make_shared<VictorValidator>(si_, victor_model));
    simp_ = std::make_shared<og::PathSimplifier>(si_);
    victor_model_ = victor_model;

    PROFILE_REINITIALIZE(10,1000);
}

void VictorPlanner::setupSpaceInformation()
{
    std::cout << "using original setupSpaceInformation\n";
    si_->setStateValidityChecker(vv_ptr->getptr());
    si_->setMotionValidator(vv_ptr->getptr());
    si_->setup();
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::planPath(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    preparePlanner(start, goal);
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
    
    PROFILE_REINITIALIZE(10,1000);

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
        int nd = stateSpace->validSegmentCount(s1, s2)*10;
        
        for(int j = 0; j<nd; j++)
        {
            stateSpace->interpolate(s1, s2, (double)j / (double)nd, state);
            const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            std::vector<double> d_state;
            d_state.insert(d_state.end(), &values[0], &values[7]);
            d_path.push_back(d_state);
        }
    }
    si_->freeState(state);
    
    return d_path;

}

Maybe::Maybe<Path> VictorPlanner::planPathConfig(VictorConfig start, VictorConfig goal)
{
    return planPathDouble(victor_model_->toValues(start), victor_model_->toValues(goal));
}

Maybe::Maybe<Path> VictorPlanner::planPathDouble(std::vector<double> start, std::vector<double> goal)
{
    ob::ScopedState<ob::RealVectorStateSpace> start_ss(space);
    ob::ScopedState<> goal_ss(space);
    std::cout << "start: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << start[i] << ", ";
        start_ss[i] = start[i];
        goal_ss[i] = goal[i];
    }
    std::cout << "\n";
    std::cout << "goal: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << goal[i] << ", ";
    }
    std::cout << "\n";
    Maybe::Maybe<ob::PathPtr> ompl_path = planPath(start_ss, goal_ss);
    if(!ompl_path.Valid())
    {
        return Maybe::Maybe<Path>();
    }
    return Maybe::Maybe<Path>(omplPathToDoublePath(ompl_path.Get()->as<og::PathGeometric>()));
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

void VictorLBKPiece::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner_->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goal);

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
    pv_ = std::make_shared<VictorPathValidator>(si_, victor_model_);
    lrrtf->setPathValidator(pv_);
    planner_ = lrrtf;
    planner_->setup();
    threshold = 0.5;
}

Maybe::Maybe<ob::PathPtr> VictorLazyRRTF::planPath(ompl::base::ScopedState<> start,
                                                   ompl::base::ScopedState<> goal)
{
    std::cout << "Using this planner\n";
    preparePlanner(start, goal);
    ob::PathPtr path;
    int planning_time = 5;

    pv_->setProbabilityThreshold(threshold);
    ob::PlannerStatus solved = planner_->solve(planning_time);

    while(!solved)
    {
        threshold += 0.1;
        std::cout << "threshold " << threshold << "\n";
        pv_->setProbabilityThreshold(threshold);
        solved = planner_->solve(planning_time);
        if(threshold > 1.2){
            break;
        }
    }
    threshold -= 0.1;

    
    if (!solved)
    {
        std::cout << "No solution could be found" << std::endl;
        return Maybe::Maybe<ob::PathPtr>();
    }

    path = pdef_->getSolutionPath();

    return Maybe::Maybe<ob::PathPtr>(path);
}


void VictorLazyRRTF::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    pdef_->setStartAndGoalStates(start, goal);
    planner_->setProblemDefinition(pdef_);

}



