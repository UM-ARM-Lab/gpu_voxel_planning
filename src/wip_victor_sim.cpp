#include "gpu_voxels_victor.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>


std::shared_ptr<SimWorld> sim_world;




/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int)
{
    std::cout << "Resetting\n";
    sim_world->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    sim_world->gvl.reset();
    exit(EXIT_SUCCESS);
}




int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    sim_world = std::make_shared<SimWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    sim_world->initializeObstacles();

    std::vector<double> start = {-1.4, 1.4, 1.4, -0.5, 0, 0, 0};
    std::vector<double> end = {0, 1.2, 0, 0, 0, 0, 0};

    sim_world->victor_model.updateActual(sim_world->victor_model.toVictorConfig(start.data()));
    
    int unused;
    std::cout << "Waiting for user input to start...\n";
    std::cin >> unused;



    Path path;
    std::vector<double> cur = start;
    for(int i=0; i<100; i++)
    {
        for(size_t j=0; j<cur.size(); j++)
        {
            cur[j] += (end[j] - start[j])/100;
        }
        path.push_back(cur);
    }

    size_t last_valid;
    if(!sim_world->executePath(path, last_valid))
    {
        std::cout << "backing up\n";
        Path backup;
        for(size_t i = last_valid; (i >= last_valid - 10) && (i != (size_t)-1); i--)
        {
            backup.push_back(path[i]);
        }
        sim_world->executePath(backup, last_valid);
    }


    // VictorPlanner planner(sim_world.get());
    // VictorLBKPIECE planner_unused0(sim_world.get());
    // VictorRRTstar<VictorMinVoxObjective> planner_unused1(sim_world.get());
    // VictorRRTstar<VictorMinColProbObjective> planner_unused2(sim_world.get());
    // VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(sim_world.get());
    // VictorTRRT<VictorMinColProbSweptObjective> planner_unused4(sim_world.get());

    // VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(sim_world.get());
    // VictorTRRT<VictorMinColProbSweptObjective> planner(sim_world.get());
    // VictorLazyRRTF planner(sim_world.get());

    // std::vector<double> start = {0.5, 0.2, 0.5};
    // std::vector<double> goal = {0.5, 0.8, 0.5};

    // bool reached_goal = false;

    // while(!reached_goal)
    // {
    //     sim_world->updateActual(Victor(start[0], start[1], start[2], VICTOR_WIDTH));
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

    sim_world->gvl.reset();
    
}
