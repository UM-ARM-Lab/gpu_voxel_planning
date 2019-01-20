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
using namespace arc_dijkstras;
using namespace PathUtils;
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
int attemptStep(const std::vector<int64_t> &full_path, HaltonGraph &g, GpuVoxelsVictor* victor)
{
    GraphEdge &e = g.GetNodeMutable(full_path[0]).GetEdgeMutable(full_path[1]);
    assert(e.GetValidity() != EDGE_VALIDITY::INVALID);
    
    // if(e.validity == EDGE_VALIDITY::UNKNOWN)
    // {
    //     checkEdge(e, g, victor);
    // }

    auto start = g.GetNodeMutable(full_path[0]).GetValueMutable();
    auto end = g.GetNodeMutable(full_path[1]).GetValueMutable();


    double w = g.GetNodeMutable(full_path[0]).GetEdgeMutable(full_path[1]).GetWeight();

    std::cout << "Attempting edge with weight " << w << "\n";
    
    Path path{start, end};
    bool valid = g_sim_world->attemptPath(path);
    e.SetValidity(valid ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
    
    return e.GetValidity() == EDGE_VALIDITY::VALID ? full_path[1] : full_path[0];
}



void showVert(Roadmap &rm)
{
    // ros::Rate r(1);
    for(auto n: rm.GetNodesMutable())
    {
        VictorConfig c = g_sim_world->victor_model.toVictorConfig(n.GetValueImmutable().data());
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

    // while(true)
    // {
        for(int i=0; i<path_ind.size()-1; i++)
        {
            auto p1 = rm.GetNodeImmutable(path_ind[i]).GetValueImmutable();
            auto p2 = rm.GetNodeImmutable(path_ind[i+1]).GetValueImmutable();
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
    // }
}



int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    g_sim_world = new SimTable;
    GpuVoxelsVictor &vm = g_sim_world->victor_model;
    // g_sim_world = new SimWall;
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    std::string graph_filename = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph";
    // std::string graph_filename = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/tmp.graph";
    
    // Roadmap rm_orig = Roadmap(&vm);
    // rm_orig.saveToFile(graph_filename);

    Roadmap rm = Roadmap(&vm, graph_filename);
    
    int start_ind = rm.addVertexAndEdges(vm.toValues(g_sim_world->init_config));
    int goal_ind = rm.addVertexAndEdges(vm.toValues(g_sim_world->goal_config));
    // showVert(rm);
    // showPath(rm, start_ind, goal_ind);
    attemptBestPath(rm, start_ind, goal_ind);


    // g_sim_world->gvl.reset();
    
}
