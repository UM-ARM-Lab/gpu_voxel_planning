#include "victor_planning.hpp"

#include <signal.h>
#include <iostream>

#include <gpu_voxels/logging/logging_gpu_voxels.h>

#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <ompl/geometric/PathSimplifier.h>


#include <stdlib.h>

#include <memory>


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;


const std::string FULL_PLANNING_NAME = "Full Planning";
const std::string PLANNING_NAME = "unsmoothed planning";
const std::string SMOOTHING_NAME = "smoothing";

const std::string ISVALID_INSERTION = "isValid insertion";
const std::string QUERY_INSERTION = "query insertion";
const std::string ISVALID_COLLISION_TEST = "isValid collision test";
const std::string CHECK_MOTION_SIMPLE_INSERTION = "checkMotion (simple) insertion";
const std::string CHECK_MOTION_SIMPLE_COLLISION_TEST = "checkMotion (simple) collision test";
const std::string CHECK_MOTION_SIMPLE_CHECK = "checkMotion (simple) full check";
const std::string CHECK_MOTION_COMP_CHECK = "checkMotion (complicated) full check";


std::shared_ptr<VictorValidator> vv_ptr;

// We define exit handlers to quit the program:
void ctrlchandler(int)
{
    // TODO: Find what is actually holding onto the ptr

    // Something here is holding onto the shared_ptr stored in vv_ptr, thus
    // even when quitting the program, memory stays allocated.
    // explicitly calling the destructor frees the memory, but this should instead
    // be done when all references to the shared_ptr leave.

    vv_ptr->~VictorValidator();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    // TODO: Find what is actually holding onto the ptr
    vv_ptr->~VictorValidator();
    exit(EXIT_SUCCESS);
}

VictorPlanner::VictorPlanner()
{
    space = std::make_shared<ob::RealVectorStateSpace>(7);

    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
    space->setBounds(bounds);
    si_ = std::make_shared<ob::SpaceInformation>(space);
    vv_ptr = std::shared_ptr<VictorValidator>(std::make_shared<VictorValidator>(si_));

    simp_ = std::make_shared<og::PathSimplifier>(si_);
    si_->setStateValidityChecker(vv_ptr->getptr());
    si_->setMotionValidator(vv_ptr->getptr());
    si_->setup();



    PROFILE_RESET_ALL(10,1000);
    // planner = std::make_shared<og::LBKPIECE1>(si_);
    // planner = std::make_shared<og::TRRT>(si_);
    // setup_planner(si_);
}

ob::PathPtr VictorPlanner::planPath(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    // pdef_->setOptimizationObjective();
    prepare_planner(start, goal);

    // planner_->setup();

    PROFILE_START(FULL_PLANNING_NAME);
    PROFILE_START(PLANNING_NAME);
    

    // ob::PlannerStatus solved = planner_->ob::Planner::solve(20.0);
    ob::PlannerStatus solved = planner_->solve(10);

    
    PROFILE_RECORD(PLANNING_NAME);

    
    ob::PathPtr path;

    //If a solution has been found, we simplify and display it.
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        path = pdef_->getSolutionPath();
        std::cout << "Found solution. Simplifying..." << std::endl;
        // print the path to screen
        // path->print(std::cout);

        PROFILE_START(SMOOTHING_NAME);
        simp_->simplifyMax(*(path->as<og::PathGeometric>()));
        PROFILE_RECORD(SMOOTHING_NAME);

        // std::cout << "Simplified solution:" << std::endl;
        // print the path to screen
        // path->print(std::cout);

        vv_ptr->visualizeSolution(path);

    }else{
        std::cout << "No solution could be found" << std::endl;
    }
    
    PROFILE_RECORD(FULL_PLANNING_NAME);

    std::vector<std::string> summary_names = {
        FULL_PLANNING_NAME,
        PLANNING_NAME,
        SMOOTHING_NAME,
        ISVALID_INSERTION,
        ISVALID_COLLISION_TEST,
        CHECK_MOTION_SIMPLE_INSERTION,
        CHECK_MOTION_SIMPLE_COLLISION_TEST,
        CHECK_MOTION_SIMPLE_CHECK,
        CHECK_MOTION_COMP_CHECK};
        
    PROFILE_PRINT_SUMMARY_FOR_GROUP(summary_names);
    // PROFILE_PRINT_SUMMARY_FOR_SINGLE(FULL_PLANNING_NAME);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(PLANNING_NAME);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(SMOOTHING_NAME);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(ISVALID_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(QUERY_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(ISVALID_COLLISION_TEST);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_COLLISION_TEST);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_CHECK);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_COMP_CHECK);

    
    PROFILE_RESET_ALL(10,1000);

    return path;
}

ob::PathPtr VictorPlanner::planPath(std::vector<double> start, std::vector<double> goal)
{
    ob::ScopedState<> start_ss(space);
    ob::ScopedState<> goal_ss(space);
    std::cout << "start: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << start[i] << ", ";
        start_ss[i] = start[i];
        goal_ss[i] = goal[i];
    }
    std::cout << "\n";
    return planPath(start_ss, goal_ss);
}



