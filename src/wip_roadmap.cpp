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
#include "lazysp_voxels.hpp"

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



/*
 *  Attempts a single step along the path and returns teh robot location
 */
int attemptStep(const std::vector<int> &full_path, Graph &g, GpuVoxelsVictor* victor)
{
    Edge &e = g.getEdge(full_path[0], full_path[1]);
    assert(e.validity != EDGE_VALIDITY::INVALID);
    
    // if(e.validity == EDGE_VALIDITY::UNKNOWN)
    // {
    //     checkEdge(e, g, victor);
    // }

    auto start = g.V[full_path[0]].q;
    auto end = g.V[full_path[1]].q;

    
    Path path{start, end};
    bool valid = g_sim_world->attemptPath(path);
    e.validity = valid ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
    
    return e.validity == EDGE_VALIDITY::VALID ? full_path[1] : full_path[0];
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

void attemptBestPath(Roadmap &rm, int start, int goal)
{
    waitForKeypress();
    while(start != goal)
    {
        // auto path_ind = A_star(start, goal, rm);
        auto path_ind = planPath(start, goal, rm, &(g_sim_world->victor_model));
        start = attemptStep(path_ind, rm, &(g_sim_world->victor_model));
    }
}

void showPath(Roadmap &rm, int start, int goal)
{
    waitForKeypress();
    // auto path_ind = A_star(start, goal, rm);
    auto path_ind = planPath(start, goal, rm, &(g_sim_world->victor_model));

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
    // showPath(rm, start_ind, goal_ind);
    attemptBestPath(rm, start_ind, goal_ind);


    // g_sim_world->gvl.reset();
    
}
