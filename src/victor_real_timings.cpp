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

typedef std::vector<std::vector<double>> dGoals;

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
        if(std::fabs(cur_values[i] - goal[i]) > 0.03)
        {
            return false;
        }
    }
    return true;
}

bool checkAtGoal(dGoals goals)
{
    for(auto goal: goals)
    {
        if(checkAtGoal(goal))
        {
            return true;
        }
    }
    return false;
}


VictorConfig getMinGoal(VictorPlanner &planner, dGoals goals)
{
    // double cost = std::numeric_limits<double>::max();
    // size_t best_goal_ind = 0;
    // for(auto goal: goals)
    // {
    //     VictorConfig c = real_world->victor_model.toVictorConfig(goal.data());
    //     planner.vppc
    // }
    return real_world->victor_model.toVictorConfig(goals[0].data());
}


bool attemptGoal(VictorPlanner &planner, dGoals goals)
{

    VictorConfig goal_config; 

    bool reached_goal = false;
    arc_utilities::Stopwatch stopwatch;
    double timeout = 60* 5; //seconds;
    double planning_iters=0;
    std::string planning_iters_name = planner.name + " plan iters";
    PROFILE_START(planning_iters_name);
    PROFILE_START(planner.name + " success");
    PROFILE_START(planner.name + " failure");
    while(!reached_goal)
    {
        if(stopwatch() > timeout)
        {
            break;
        }
        goal_config = getMinGoal(planner, goals);
        
        std::cout << "Control + Plan iter " << planning_iters << "\n";
        if(DO_CONTROL)
        {
            
            PROFILE_START(planner.name + " control");
            Optpath maybe_path = planner.localControlConfig(real_world->victor_model.cur_config,
                                                            goal_config);
            while(maybe_path.Valid())
            {
                goal_config = getMinGoal(planner, goals);
                if(stopwatch() > timeout) break;
                
                std::cout << "Local control found, executing\n";
                real_world->attemptPath(maybe_path.Get());
                reached_goal = checkAtGoal(goals);
                // std::cout << "Goal reached? " << reached_goal << "\n";
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
            goal_config = getMinGoal(planner, goals);
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
    {
        std::cout << "\n\nGoal Reached!\n\n\n";
        PROFILE_RECORD(planner.name + " success");
    }
    else
    {
        std::cout << "\n\nExiting, but goal not reached\n\n\n";
        PROFILE_RECORD(planner.name + " failure");
    }
    real_world->spinUntilUpdate();
    return true;
}


void setupWorld()
{
    if(real_world != nullptr)
    {
        real_world->gvl.reset();
        real_world->victor_model.gvl.reset();
    }

    real_world = std::make_shared<RealWorld>();

    ros::Duration(0.5).sleep();
    ros::spinOnce();

}

bool moveToStart(std::vector<double> start)
{
    Path path;
    path.push_back(start);
    bool at_start = false;
    while(!at_start)
    {
        std::cout << "Moving toward start";
        at_start = real_world->attemptPath(path);
        if(!at_start)
        {
            std::cout << "Failed to reach startpoint. Enter input to retry\n";
            std::string unused;
            std::cin >> unused;
        }
    }
}


void runTest_VictorProbCol(std::vector<double> start, dGoals goals)
{
    std::cout << "Waiting for input to start prob planner...\n";
    std::string unused;
    std::cin >> unused;


    setupWorld();
    moveToStart(start);

    std::cout << "Set up Visualizer now...\n";
    std::cin >> unused;

    VictorProbColCostRRTConnect planner(&(real_world->victor_model));
    planner.name = "prob_col_planner";
    planner.use_anytime_planner = false;
    std_msgs::String msg;
    ros::Duration(1.0).sleep();
    attemptGoal(planner, goals);
}

void runTest_VictorVox(std::vector<double> start, dGoals goals)
{
    std::cout << "Waiting for input to start vox planner...\n";
    std::string unused;
    // std::getline(std::cin, unused);
    std::cin >> unused;

    setupWorld();
    moveToStart(start);

    std::cout << "Set up Visualizer now...\n";
    std::cin >> unused;

    VictorVoxCostRRTConnect planner(&(real_world->victor_model));
    planner.name = "vox_planner";
    std_msgs::String msg;
    ros::Duration(1.0).sleep();
    attemptGoal(planner, goals);
}

int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "gpu_voxels");
    ros::NodeHandle nh;
    speaker = nh.advertise<std_msgs::String>("/polly", 1, true);


    

    std::vector<double> start = {-1.063, 1.558, -0.779, 0.022, 1.042, -0.46, -0.715};
    std::vector<double> goal_box = {-0.757, 0.89, -0.571, 0.555, 1.046, -0.793, -0.679};

    dGoals goals;
    goals.push_back(goal_box);

    int num_trials = 1;
    for(int i=0; i<num_trials; i++)
    {
        std::cout << "\n\n\n\nTrial " << i+1 << " of " << num_trials << "\n\n\n\n\n";
        runTest_VictorProbCol(start, goals);
        runTest_VictorVox(start, goals);
    }

    std::string filename = "./real_robot_trials/box_" + arc_helpers::GetCurrentTimeAsString();
    
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
    std::cout << "\n\n\n\n\n\nWrote summary!!\n\n\n\n\n\n";
    real_world->gvl.reset();
    
}
