#include "gpu_voxels_victor.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>



std::shared_ptr<SimWorld> simWorld;




/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int)
{
    std::cout << "Resetting\n";
    simWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    simWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}




int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    simWorld = std::make_shared<SimWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    simWorld->initializeObstacles();

    int unused;
    std::cout << "Waiting for user input to start...\n";
    std::cin >> unused;

    Path path;
    for(double d = 0.0; d < 1.0; d+=0.02)
    {
        path.push_back(std::vector<double>(7,d));
    }

    simWorld->executePath(path);

    // VictorPlanner planner(simWorld.get());
    // VictorLBKPIECE planner_unused0(simWorld.get());
    // VictorRRTstar<VictorMinVoxObjective> planner_unused1(simWorld.get());
    // VictorRRTstar<VictorMinColProbObjective> planner_unused2(simWorld.get());
    // VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(simWorld.get());
    // VictorTRRT<VictorMinColProbSweptObjective> planner_unused4(simWorld.get());

    // VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(simWorld.get());
    // VictorTRRT<VictorMinColProbSweptObjective> planner(simWorld.get());
    // VictorLazyRRTF planner(simWorld.get());

    // std::vector<double> start = {0.5, 0.2, 0.5};
    // std::vector<double> goal = {0.5, 0.8, 0.5};

    // bool reached_goal = false;

    // while(!reached_goal)
    // {
    //     simWorld->updateActual(Victor(start[0], start[1], start[2], VICTOR_WIDTH));
    //     Maybe::Maybe<ob::PathPtr> path = planner.planPathDouble(start, goal);
    //     if(!path.Valid())
    //     {
    //         std::cout << "Path planning failed\n";
    //         break;
    //     }
    //     reached_goal = planner.executePath(path.Get()->as<og::PathGeometric>());
    // }

    // if(reached_goal)
    //     std::cout << "REACHED GOAL!\n";

    simWorld->gvl.reset();
    
}
