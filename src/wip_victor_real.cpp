#include <ros/ros.h>
#include "gpu_voxels_victor.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>

#include <iostream>
#include <fstream>


std::shared_ptr<RealWorld> real_world;


using namespace gpu_voxels_planner;
namespace M = Maybe;


/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int)
{
    std::cout << "Resetting\n";
    real_world->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    real_world->gvl.reset();
    exit(EXIT_SUCCESS);
}


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal)
{
    VictorConfig goal_config = real_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;

    while(!reached_goal)
    {
        M::Maybe<Path> maybe_path = planner.planPathConfig(real_world->victor_model.cur_config,
                                                    goal_config);
        if(!maybe_path.Valid())
        {
            std::cout << "Path planning failed\n";
            return false;
        }
        reached_goal = real_world->attemptPath(maybe_path.Get());
    }
    return true;
}


int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "gpu_voxels");
    
    real_world = std::make_shared<RealWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    VictorProbColCostRRTConnect planner(&(real_world->victor_model));

    std::vector<double> goal = {-.25, -.11, -.18, -1.0, .4, .4, -.7};

    std::string unused;
    std::cout << "Waiting for user input to start...\n";
    std::getline(std::cin, unused);
    while(ros::ok())
    {
        ros::spinOnce();
    }


    attemptGoal(planner, goal);
    

    real_world->gvl.reset();
    
}
