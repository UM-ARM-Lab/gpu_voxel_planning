#include "gpu_voxels_victor.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>

#include <iostream>
#include <fstream>


std::shared_ptr<SimWorld> sim_world;


using namespace gpu_voxels_planner;
namespace M = Maybe;


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


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal)
{
    VictorConfig goal_config = sim_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;

    while(!reached_goal)
    {
        M::Maybe<Path> maybe_path = planner.planPathConfig(sim_world->victor_model.cur_config,
                                                    goal_config);
        if(!maybe_path.Valid())
        {
            std::cout << "Path planning failed\n";
            return false;
        }
        reached_goal = sim_world->attemptPath(maybe_path.Get());
    }
    return true;
}


void wiggleFingers()
{

    VictorConfig right_gripper_config;

    while(true)
    {
        for(double p=-1.5; p<1.5; p+=0.1)
        {
            right_gripper_config["victor_right_gripper_fingerA_joint_2"] = p;
            right_gripper_config["victor_right_gripper_fingerB_joint_2"] = p;
            right_gripper_config["victor_right_gripper_fingerC_joint_2"] = p;
            sim_world->victor_model.updateActual(right_gripper_config);
            sim_world->victor_model.doVis();
            usleep(100000);
            std::cout << "fingers at " << p << "\n";
            
        }
        for(double p=1.5; p>-1.5; p-=0.1)
        {
            right_gripper_config["victor_right_gripper_fingerA_joint_2"] = p;
            right_gripper_config["victor_right_gripper_fingerB_joint_2"] = p;
            right_gripper_config["victor_right_gripper_fingerC_joint_2"] = p;
            sim_world->victor_model.updateActual(right_gripper_config);
            sim_world->victor_model.doVis();
            usleep(100000);
            std::cout << "fingers at " << p << "\n";
        }
    }

}



void testAngles()
{
    
    while(true)
    {
        std::string unused;
        std::cout << "Waiting for user input to set position...\n";
        std::getline(std::cin, unused);

        std::ifstream myfile;
        myfile.open("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/config/test_angles.txt");
        std::cout << "file is open: " << myfile.is_open() << "\n";
        std::vector<double> goal_tmp;
        goal_tmp.resize(7);
        for(int i=0; i<7; i++)
        {
            myfile >> goal_tmp[i];
            std::cout << goal_tmp[i] << ", ";
        }
        std::cout << "\n";
        myfile.close();
        sim_world->victor_model.updateActual(sim_world->victor_model.toVictorConfig(goal_tmp.data()));
        sim_world->victor_model.doVis();
    }
}


int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    sim_world = std::make_shared<SimWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    sim_world->initializeObstacles();
// 
    // std::vector<double> start = {-1.4, 1.4, 1.4, -0.5, 0.01, 0.01, 0.05};
    std::vector<double> start = {0, 0, 0, 0, 0.00, 0.00, 0.00};
    std::vector<double> goal = {-0.15, 1.0, 0, -0.5, 0, 1.0, 0};

    sim_world->victor_model.updateActual(sim_world->victor_model.toVictorConfig(start.data()));

    std::string unused;
    std::cout << "Waiting for user input to start...\n";
    std::getline(std::cin, unused);

    wiggleFingers();
    testAngles();
    
    // VictorLBKPiece planner(&(sim_world->victor_model));
    // VictorThresholdRRTConnect planner(&(sim_world->victor_model));
    VictorProbColCostRRTConnect planner(&(sim_world->victor_model));
    
    // VictorPRM planner(&(sim_world->victor_model));
    // VictorLazyRRTF planner(&(sim_world->victor_model));

    attemptGoal(planner, goal);
    
    // M::Maybe<Path> maybe_path = planner.planPathDouble(start, goal);

    // if(!maybe_path.Valid())
    // {
    //     std::cout << "no path found\n";
    //     return 0;
    // }
    // std::cout << "Path found\n";
    // Path path = maybe_path.Get();
    // sim_world->attemptPath(path);



    sim_world->gvl.reset();
    
}
