#include "gpu_voxels_victor.hpp"
#include "worlds.hpp"
#include "victor_planning.hpp"
#include "victor_local_controller.hpp"
#include "helpers.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>

#include <arc_utilities/timing.hpp>
#include "victor_halton_roadmap.hpp"

#include <iostream>
#include <fstream>

#define PROB_OCCUPIED eBVM_OCCUPIED


SimWorld* g_sim_world;


using namespace gpu_voxels_planner;
namespace M = Maybe;


/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int)
{
    std::cout << "Resetting\n";
    g_sim_world->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    g_sim_world->gvl.reset();
    exit(EXIT_SUCCESS);
}


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal)
{
    VictorConfig goal_config = g_sim_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;

    while(!reached_goal)
    {
        M::Maybe<Path> maybe_path = planner.planPathConfig(g_sim_world->victor_model.cur_config,
                                                    goal_config);
        if(!maybe_path.Valid())
        {
            std::cout << "Path planning failed\n";
            return false;
        }
        reached_goal = g_sim_world->attemptPath(maybe_path.Get());
    }
    return true;
}


void showVert(Roadmap &rm)
{
    // ros::Rate r(1);
    for(auto v: rm.V)
    {
        VictorConfig c = g_sim_world->victor_model.toVictorConfig(v.q.data());
        g_sim_world->victor_model.updateActual(c);
        g_sim_world->victor_model.doVis();
        usleep(100000);
    }
}

void showPath(Roadmap &rm, int start, int goal)
{
    auto path_ind = A_star(start, goal, rm);

    while(true)
    {
        for(int i=0; i<path_ind.size()-1; i++)
        {
            auto p1 = rm.V[path_ind[i]].q;
            auto p2 = rm.V[path_ind[i+1]].q;
            auto path = interpolatePath(p1, p2, 0.1);

            for(auto &point: path)
            {
                VictorConfig c = g_sim_world->victor_model.toVictorConfig(point.data());
                g_sim_world->victor_model.updateActual(c);
                g_sim_world->victor_model.doVis();
                usleep(100000);
            }
        }
        // for(auto ind: path_ind)
        // {
        //     VictorConfig c = g_sim_world->victor_model.toVictorConfig(rm.V[ind].q.data());
        //     g_sim_world->victor_model.updateActual(c);
        //     g_sim_world->victor_model.doVis();
        //     usleep(1000000);
        // }
    }
}



int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    g_sim_world = new SimTable;
    GpuVoxelsVictor &vm = g_sim_world->victor_model;
    // g_sim_world = new SimWall;
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    Roadmap rm = Roadmap(&vm);
    int start_ind = rm.insertVertex(vm.toValues(g_sim_world->init_config));
    int goal_ind = rm.insertVertex(vm.toValues(g_sim_world->goal_config));
    // showVert(rm);
    showPath(rm, start_ind, goal_ind);


    // g_sim_world->gvl.reset();
    
}
