#include "victor_planning.hpp"

#include <signal.h>
#include <iostream>

#include <gpu_voxels/logging/logging_gpu_voxels.h>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>


#include <ompl/geometric/PathSimplifier.h>


#include <stdlib.h>

#include <memory>


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;

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



    // planner = std::make_shared<og::LBKPIECE1>(si_);
    // planner = std::make_shared<og::TRRT>(si_);
    // setup_planner(si_);
}

ob::PathPtr VictorPlanner::planPath(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    // pdef_->setOptimizationObjective();
    prepare_planner(start, goal);

    // planner_->setup();
    
    PERF_MON_START("planner");

    ob::PlannerStatus solved = planner_->ob::Planner::solve(20.0);
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");
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

        PERF_MON_START("simplify");
        simp_->simplifyMax(*(path->as<og::PathGeometric>()));
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("simplify", "Simplification time", "planning");

        // std::cout << "Simplified solution:" << std::endl;
        // print the path to screen
        // path->print(std::cout);

        vv_ptr->visualizeSolution(path);

    }else{
        std::cout << "No solution could be found" << std::endl;
    }

    
    PERF_MON_SUMMARY_PREFIX_INFO("planning");

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



