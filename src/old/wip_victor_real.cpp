#include <ros/ros.h>
#include "std_msgs/String.h"
#include "gpu_voxels_victor.hpp"
#include "worlds.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <iostream>
#include <fstream>



#define DO_CONTROL true
#define DO_PLAN true

std::shared_ptr<RealWorld> real_world;


using namespace gpu_voxels_planner;
namespace M = Maybe;
typedef Maybe::Maybe<Path> Optpath;

ros::Publisher speaker;


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


bool checkAtGoal(std::vector<double> goal)
{
    std::vector<double> cur_values = real_world->victor_model.toValues(real_world->victor_model.cur_config);
    
    for(size_t i=0; i<goal.size(); i++)
    {
        if(std::fabs(cur_values[i] - goal[i]) > 0.05)
        {
            return false;
        }
    }
    return true;
}


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal)
{

    VictorConfig goal_config = real_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;
    arc_utilities::Stopwatch stopwatch;
    double timeout = 60* 15; //seconds;
    double planning_iters=0;
    std::string planning_iters_name = planner.name + " plan iters";
    PROFILE_START(planning_iters_name);
    while(!reached_goal)
    {
        if(DO_CONTROL)
        {
            PROFILE_START(planner.name + " control");
            Optpath maybe_path = planner.localControlConfig(real_world->victor_model.cur_config,
                                                            goal_config);
            while(maybe_path.Valid())
            {
                if(stopwatch() > timeout) break;
                
                std::cout << "Local control found, executing\n";
                real_world->attemptPath(maybe_path.Get());
                reached_goal = checkAtGoal(goal);
                if(reached_goal){
                    break;
                }

                maybe_path = planner.localControlConfig(real_world->victor_model.cur_config,
                                                        goal_config);
            }
            PROFILE_RECORD(planner.name + " control");
            if(reached_goal)
                break;
        }

        if(DO_PLAN)
        {
            planning_iters++;
            PROFILE_START(planner.name + " plan");
            Optpath maybe_path = planner.planPathConfig(real_world->victor_model.cur_config,
                                                        goal_config);
            PROFILE_RECORD(planner.name + " plan");
            if(!maybe_path.Valid())
            {
                std::cout << "Path planning failed\n";
                PROFILE_START(planner.name + "_failed");
                PROFILE_RECORD(planner.name + "_failed");
                return false;
            }


            PROFILE_START(planner.name + " execute");
            reached_goal = real_world->attemptPath(maybe_path.Get());
            PROFILE_RECORD(planner.name + " execute");
        }

        // M::Maybe<Path> maybe_path = planner.planPathConfig(real_world->victor_model.cur_config,
        //                                             goal_config);
        // if(!maybe_path.Valid())
        // {
        //     std::cout << "Path planning failed\n";
        //     return false;
        // }
        // reached_goal = real_world->attemptPath(maybe_path.Get());
    }
    PROFILE_RECORD_DOUBLE(planning_iters_name, planning_iters);
    if(reached_goal)
        std::cout << "Goal Reached!\n";
    else
        std::cout << "Exiting, but goal not reached\n";
    return true;
}


void runTest_VictorProbCol(std::vector<double> goal)
{
    VictorProbColCostRRTConnect planner(&(real_world->victor_model));
    planner.name = "prob_col_planner";
    planner.use_anytime_planner = false;
    std_msgs::String msg;
    std::cout << "Waiting for input to start prob planner...\n";
    std::string unused;
    std::getline(std::cin, unused);
    ros::Duration(1.0).sleep();
    attemptGoal(planner, goal);
}

void runTest_VictorVox(std::vector<double> goal)
{

    VictorVoxCostRRTConnect planner(&(real_world->victor_model));
    planner.name = "vox_planner";
    std_msgs::String msg;
    std::cout << "Waiting for input to start vox planner...\n";
    std::string unused;
    std::getline(std::cin, unused);
    ros::Duration(1.0).sleep();
    attemptGoal(planner, goal);
}

int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "gpu_voxels");
    ros::NodeHandle nh;
    speaker = nh.advertise<std_msgs::String>("/polly", 1, true);
    
    real_world = std::make_shared<RealWorld>();

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    //above table
    //std::vector<double> goal_box = {-0.145, 0.438, 0.242, -1.164, 0.495, 0.687, -0.804};

    //below table
    // std::vector<double> goal_box = {-1.222, 1.031, 0.984, -0.11, 0.577, 0.246, -0.342};

    std::vector<double> goal_box = {0.047, 0.331, -0.16, -1.286, -0.613, 1.242, 2.02};


    std::string unused;
    std::cout << "Waiting for user input to start...\n";
    std::getline(std::cin, unused);

    runTest_VictorProbCol(goal_box);
    // runTest_VictorVox(goal_box);

    std::string filename = "./real_robot_trials/box_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
    
    // real_world->gvl.reset();
}
