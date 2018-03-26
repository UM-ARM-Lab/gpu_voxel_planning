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



    PROFILE_REINITIALIZE(10,1000);
    // planner = std::make_shared<og::LBKPIECE1>(si_);
    // planner = std::make_shared<og::TRRT>(si_);
    // setup_planner(si_);
}

ob::PathPtr VictorPlanner::planPath(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    // pdef_->setOptimizationObjective();
    prepare_planner(start, goal);

    // planner_->setup();

    PROFILE_START(FULL_PLANNING);
    PROFILE_START(PLANNING);
    
    ob::PlannerStatus solved = planner_->solve(30);

    PROFILE_RECORD(PLANNING);
    PROFILE_START(POST_PROCESSING);
    
    ob::PathPtr path;
    std::cout << solved << "\n";
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
        vv_ptr->visualizeSolution(path);
        PROFILE_RECORD(VISUALIZE_SOLUTION);

        std::cout << "Calling post planning actions\n";
        post_planning_actions(path);

    }else{
        std::cout << "No solution could be found" << std::endl;
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
        CHECK_MOTION_COMP_CHECK};
        
    PROFILE_PRINT_SUMMARY_FOR_GROUP(summary_names);
    // PROFILE_PRINT_SUMMARY_FOR_SINGLE(FULL_PLANNING);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(PLANNING);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(SMOOTHING);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(ISVALID_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(QUERY_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(ISVALID_COLLISION_TEST);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_INSERTION);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_COLLISION_TEST);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_SIMPLE_CHECK);
    // PROFILE_PRINT_SUMMARY_FOR_GROUP(CHECK_MOTION_COMP_CHECK);

    
    PROFILE_REINITIALIZE(10,1000);

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
    std::cout << "goal: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << goal[i] << ", ";
    }
    std::cout << "\n";
    return planPath(start_ss, goal_ss);
}



