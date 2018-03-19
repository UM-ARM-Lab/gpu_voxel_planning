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
    si = std::make_shared<ob::SpaceInformation>(space);
    vv_ptr = std::shared_ptr<VictorValidator>(std::make_shared<VictorValidator>(si));

    simp = std::make_shared<og::PathSimplifier>(si);
    si->setStateValidityChecker(vv_ptr->getptr());
    si->setMotionValidator(vv_ptr->getptr());
    si->setup();



    // planner = std::make_shared<og::LBKPIECE1>(si);
    planner = std::make_shared<og::TRRT>(si);
}

ob::PathPtr VictorPlanner::planPath(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    vv_ptr->insertStartAndGoal(start, goal);
    vv_ptr->doVis();
    pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);
    // pdef->setOptimizationObjective();

    planner->clear();        
    planner->setProblemDefinition(pdef);

    // planner->setup();
    
    PERF_MON_START("planner");
    planner->clear(); // this clears all roadmaps
    ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");
    ob::PathPtr path;

    //If a solution has been found, we simplify and display it.
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        path = pdef->getSolutionPath();
        std::cout << "Found solution. Simplifying..." << std::endl;
        // print the path to screen
        // path->print(std::cout);

        PERF_MON_START("simplify");
        simp->simplifyMax(*(path->as<og::PathGeometric>()));
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

int main(int argc, char **argv)
{
    // Always initialize our logging framework as a first step, with the CLI arguments:
    icl_core::logging::initialize(argc, argv);
    
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    PERF_MON_INITIALIZE(100, 1000);
    // PERF_MON_ENABLE("planning");

    // std::cout << "Prob: " << uint32_t(BitVoxelMeaning(255)) << "\n";
    std::cout << "Prob: " << (int) MAX_PROBABILITY << "\n";
    std::cout << "Prob: " << (int) Probability (int32_t((const uint32_t)BitVoxelMeaning(255))+int32_t(UNKNOWN_PROBABILITY)) << "\n";
    std::cout << "Equal?: " << (MAX_PROBABILITY == Probability (int32_t(BitVoxelMeaning(255))+int32_t(UNKNOWN_PROBABILITY))) << "\n";


    VictorPlanner vpln = VictorPlanner();


    //Create a random start state:
    // ob::ScopedState<> start(vpln.space);
    // start[0] = -1.3;
    // start[1] = -0.2;
    // start[2] = 0.0;
    // start[3] = 0.0;
    // start[4] = 0.0;
    // start[5] = 0.0;
    // start[6] = 0.0;
    // //And a random goal state:
    // ob::ScopedState<> goal(vpln.space);
    // goal[0] = 1.3;
    // goal[1] = -0.5;
    // goal[2] = 0.0;
    // goal[3] = 0.0;
    // goal[4] = 0.0;
    // goal[5] = 0.0;
    // goal[6] = 0.0;
    std::vector<double> start{-1.3, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> goal{1.3, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0};

    vpln.vv_ptr->testObstacle();

    PERF_MON_START("planner");

    vpln.planPath(start, goal);

    PERF_MON_SUMMARY_PREFIX_INFO("planning");

    // keep the visualization running:
    while(true)
    {
        vpln.vv_ptr->doVis();
        usleep(30000);
    }

    return 1;
}
