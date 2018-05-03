#include <ros/ros.h>
#include "std_msgs/String.h"
#include "gpu_voxels_victor.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>
#include <arc_utilities/timing.hpp>

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


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal, std::string planner_name)
{

    VictorConfig goal_config = real_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;
    arc_utilities::Stopwatch stopwatch;
    double timeout = 60* 15; //seconds;
    while(!reached_goal)
    {
        if(DO_CONTROL)
        {
            PROFILE_START(planner_name + " control");
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
            PROFILE_RECORD(planner_name + " control");
            if(reached_goal)
                break;
        }

        if(DO_PLAN)
        {
            PROFILE_START(planner_name + " plan");
            Optpath maybe_path = planner.planPathConfig(real_world->victor_model.cur_config,
                                                        goal_config);
            PROFILE_RECORD(planner_name + " plan");
            if(!maybe_path.Valid())
            {
                std::cout << "Path planning failed\n";
                return false;
            }


            PROFILE_START(planner_name + " execute");
            reached_goal = real_world->attemptPath(maybe_path.Get());
            PROFILE_RECORD(planner_name + " execute");
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
    if(reached_goal)
        std::cout << "Goal Reached!\n";
    else
        std::cout << "Exiting, but goal not reached\n";
    return true;
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

    VictorProbColCostRRTConnect planner(&(real_world->victor_model));
    // planner.use_anytime_planner = false;

    std::vector<double> goal1 = {-.25, -.11, -.18, -1.0, .4, .4, -.7};
    std::vector<double> goal2 = {-.25, .5, .3, -1.0, .4, .4, -.7};

    std::vector<double> goal_test = {0.149, 0.712, -0.307, -0.768, 0.001, 1.238, -0.91};

    std::string unused;
    std::cout << "Waiting for user input to start...\n";
    std::getline(std::cin, unused);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    // }


    std_msgs::String msg;
    std::cout << "Waiting for input for goal test...\n";
    std::getline(std::cin, unused);
    ros::Duration(1.0).sleep();
    attemptGoal(planner, goal_test, "test_planner");



    // while(ros::ok())
    // {
    //     msg.data = "goal one";
    //     // speaker.publish(msg);



    //     std::cout << "Waiting for input for goal 1...\n";
    //     std::getline(std::cin, unused);
    //     ros::Duration(1.0).sleep();
    //     attemptGoal(planner, goal1, "test_planner");

        
    //     msg.data = "goal two";
    //     // speaker.publish(msg);
    //     std::cout << "Waiting for input for goal 2...\n";
    //     std::getline(std::cin, unused);
    //     ros::Duration(1.0).sleep();

    //     attemptGoal(planner, goal2, "test_planner");


    // }
    

    real_world->gvl.reset();
    
}
