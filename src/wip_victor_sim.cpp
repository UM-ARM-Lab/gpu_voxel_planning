#include "gpu_voxels_victor.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>


std::shared_ptr<SimWorld> sim_world;


using namespace gpu_voxels_planner;


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
    std::vector<double> goal = {0, 1.2, 0, 0, 0, 0, 0};

    sim_world->victor_model.updateActual(sim_world->victor_model.toVictorConfig(start.data()));
    
    int unused;
    std::cout << "Waiting for user input to start...\n";
    std::cin >> unused;

    
    VictorLBKPiece planner(&(sim_world->victor_model));
    
    Maybe::Maybe<Path> maybe_path = planner.planPathDouble(start, goal);

    if(!maybe_path.Valid())
    {
        std::cout << "no path found\n";
        return 0;
    }
    std::cout << "Path found\n";
    Path path = maybe_path.Get();

    // Path path;
    // std::vector<double> cur = start;
    // for(int i=0; i<100; i++)
    // {
    //     for(size_t j=0; j<cur.size(); j++)
    //     {
    //         cur[j] += (goal[j] - start[j])/100;
    //     }
    //     path.push_back(cur);
    // }

    size_t last_valid;
    if(!sim_world->executePath(path, last_valid))
    {
        std::cout << "backing up\n";
        std::cout << last_valid << "\n";
        Path backup;
        for(int i=0; i<30; i++)
        {
            backup.push_back(path[last_valid]);
            if(last_valid == 0)
            {
                break;
            }
            last_valid--;
        }
        sim_world->executePath(backup, last_valid);
    }


    sim_world->gvl.reset();
    
}
