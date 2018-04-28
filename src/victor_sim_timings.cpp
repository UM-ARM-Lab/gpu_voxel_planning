#include "gpu_voxels_victor.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <csignal>
#include <vector>
#include <cmath>
#include <limits>

#include <iostream>
#include <fstream>
#include "hardcoded_params.h"

#include "common_names.hpp"


#define ENABLE_PROFILING
#include "arc_utilities/timing.hpp"

//Avoid execution on the actual robot, used to test planner with all obstacles known



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

void confirmAtGoal(std::vector<double> goal)
{
    std::vector<double> cur_values = sim_world->victor_model.toValues(sim_world->victor_model.cur_config);
    
    for(size_t i=0; i<goal.size(); i++)
    {
        if(std::fabs(cur_values[i] - goal[i]) > 0.01)
        {
            std::cout << "Robot though goal is reached, but not actually\n";
            std::cout << "joint " << i << " is at " << cur_values[i] << " but should be at " << goal[i] << "\n";
            assert(false);
        }

    }

}


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal, std::string planner_name)
{
    VictorConfig goal_config = sim_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;

    

    arc_utilities::Stopwatch stopwatch;
    double timeout = 60* 15; //seconds;
    // double timeout = 30; //seconds;
    PROFILE_START(planner_name + " success");
    while(!reached_goal && stopwatch() < timeout)
    {
        PROFILE_START(planner_name + " plan");
        M::Maybe<Path> maybe_path = planner.planPathConfig(sim_world->victor_model.cur_config,
                                                           goal_config);
        PROFILE_RECORD(planner_name + " plan");
        if(!maybe_path.Valid())
        {
            std::cout << "Path planning failed\n";
            return false;
        }

        if(PLAN_ONLY)
        {
            reached_goal = true;
            break;
        }
        PROFILE_START(planner_name + " execute");
        reached_goal = sim_world->attemptPath(maybe_path.Get());
        PROFILE_RECORD(planner_name + " execute");
    }

    if(!reached_goal)
    {
        std::cout << "Timeout before goal reached\n";
        return false;
    }
    
    std::cout << "\n\n\n== PATH COMPLETE ==\n\n\n";

    PROFILE_RECORD(planner_name + " success");

    if(!PLAN_ONLY)
    {
        confirmAtGoal(goal);
    }
    
    return true;
}




void setupWorld()
{
    if(sim_world != nullptr)
    {
        sim_world->gvl.reset();
        sim_world->victor_model.gvl.reset();
    }

    sim_world = std::make_shared<SimWorld>();
    sim_world->initializeObstacles();
}


void runTest(VictorPlanner &planner, std::string planner_name)
{
    std::vector<double> start = {0, 0, 0, 0, 0.00, 0.00, 0.00};
    std::vector<double> goal = {-0.15, 1.0, 0, -0.5, 0, 1.0, 0};
    if(PEG_IN_HOLE)
    {
        goal = std::vector<double>{-0.15, 0.52, 0.0, -0.72, 0.0, 1.0, -2.5};
    }


    
    sim_world->victor_model.updateActual(sim_world->victor_model.toVictorConfig(start.data()));
    PROFILE_START(planner_name);
    attemptGoal(planner, goal, planner_name);
    PROFILE_RECORD(planner_name);
    setupWorld();
}


void runTest_ThresholdRRTConnect()
{
    VictorThresholdRRTConnect planner(&(sim_world->victor_model));
    runTest(planner, BiRRT_TIME);
}

void runTest_ProbColCostRRTConnect()
{
    VictorProbColCostRRTConnect planner(&(sim_world->victor_model));
    runTest(planner, PROB_COL_COST_TIME);
}

void runTest_VoxCostRRTConnect()
{
    VictorVoxCostRRTConnect planner(&(sim_world->victor_model));
    runTest(planner, VOX_COST_TIME);
}



int main(int argc, char* argv[])
{


    icl_core::logging::initialize(argc, argv);

    // sim_world = std::make_shared<SimWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    PROFILE_REINITIALIZE(20, 10000);

    setupWorld();

    int num_trials = 3;
    for(int i=0; i<num_trials; i++)
    {
        std::cout << "Trial " << i + 1<< " of " << num_trials << "\n";
        // runTest_ThresholdRRTConnect();
        runTest_ProbColCostRRTConnect();
        runTest_VoxCostRRTConnect();
    }
    


    PROFILE_WRITE_SUMMARY_FOR_ALL("victor_sim_times.txt");
    std::cout << "\n\n\n\n\n\nWrote summary!!\n\n\n\n\n\n";

    

    
    

    return 0;
    // testAngles();
    
    // VictorLBKPiece planner(&(sim_world->victor_model));
    // VictorThresholdRRTConnect planner(&(sim_world->victor_model));
    
    // VictorPRM planner(&(sim_world->victor_model));
    // VictorLazyRRTF planner(&(sim_world->victor_model));


    
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