#include "gpu_voxels_victor.hpp"
#include "worlds.hpp"
#include "victor_planning.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <arc_utilities/arc_helpers.hpp>
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
using namespace PathUtils;
typedef Maybe::Maybe<Path> Optpath;


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
            std::cout << "Robot thought goal is reached, but not actually\n";
            std::cout << "joint " << i << " is at " << cur_values[i] << " but should be at " << goal[i] << "\n";
            assert(false);
        }

    }

}

bool checkAtGoal(std::vector<double> goal)
{
    std::vector<double> cur_values = sim_world->victor_model.toValues(sim_world->victor_model.cur_config);
    
    for(size_t i=0; i<goal.size(); i++)
    {
        if(std::fabs(cur_values[i] - goal[i]) > 0.01)
        {
            // std::cout << "Goal not reached\n";
            // std::cout << "joint " << i << " is at " << cur_values[i] << " but should be at " << goal[i] << "\n";

            return false;
        }
    }
    return true;
}


bool attemptGoal(VictorPlanner &planner, std::vector<double> goal)
{
    VictorConfig goal_config = sim_world->victor_model.toVictorConfig(goal.data());

    bool reached_goal = false;

    

    arc_utilities::Stopwatch stopwatch;
    double timeout = 60* 5; //seconds;
    // double timeout = 30; //seconds;
    double num_planner_iterations;
    PROFILE_START(planner.name + " planner iters");
    PROFILE_START(planner.name + " success");
    while(!reached_goal && stopwatch() < timeout)
    {


        if(DO_CONTROL)
        {
            std::cout << "Local Control\n";
            PROFILE_START(planner.name + " control");
            Optpath maybe_path = planner.localControlConfig(sim_world->victor_model.cur_config,
                                                            goal_config);
            while(maybe_path.Valid())
            {
                if(stopwatch() > timeout) break;
                
                sim_world->attemptPath(maybe_path.Get());
                reached_goal = checkAtGoal(goal);
                if(reached_goal){
                    break;
                }

                maybe_path = planner.localControlConfig(sim_world->victor_model.cur_config,
                                                        goal_config);
            }
            PROFILE_RECORD(planner.name + " control");
            if(reached_goal)
                break;
        }

        if(DO_PLAN)
        {
            std::cout << "Attemting to plan\n";
            num_planner_iterations ++;
            PROFILE_START(planner.name + " plan");
            Optpath maybe_path = planner.planPathConfig(sim_world->victor_model.cur_config,
                                                        goal_config);
            PROFILE_RECORD(planner.name + " plan");

            PROFILE_START(planner.name + " execute");
            if(maybe_path.Valid())
            {
                reached_goal = sim_world->attemptPath(maybe_path.Get());
            }
            PROFILE_RECORD(planner.name + " execute");
        }


        // DO WIGGLE
        if(DO_RANDOM_WIGGLE)
        {
            std::string unused;
            std::getline(std::cin, unused);

            std::cout << "Random Wiggling\n";
            for(int i=0; i<50; i++)
            {
                Path wiggle_path = planner.randomWiggleConfig(sim_world->victor_model.cur_config);
                sim_world->executeAndReturn(wiggle_path);
            }
        }

        if(DO_IOU_WIGGLE)
        {
            std::cout << "IouWiggling Wiggling\n";
            for(int i=0; i<50; i++)
            {
                Path wiggle_path = planner.iouWiggleConfig(sim_world->victor_model.cur_config);
                sim_world->executeAndReturn(wiggle_path);
            }

        }

    }

    if(!reached_goal)
    {
        std::cout << "\n\n\nTimeout before goal reached\n\n\n";
        return false;
    }

    PROFILE_RECORD_DOUBLE(planner.name + " planner iters", num_planner_iterations);
    std::cout << "\n\n\n== PATH COMPLETE ==\n\n\n";

    double time = PROFILE_RECORD(planner.name + " success");
    std::cout << "Total time: " << time << "\n";

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

    // sim_world = std::make_shared<SimTable>();
    sim_world = std::make_shared<SimBookshelf>();
    // sim_world->initializeObstacles();
}


void runTest(VictorPlanner &planner)
{

    robot::JointValueMap gconfig = sim_world->goal_config;
    std::vector<double> goal = sim_world->victor_model.toValues(gconfig);

    PROFILE_START(planner.name);
    std::string unused;
    // std::getline(std::cin, unused);
    attemptGoal(planner, goal);
    PROFILE_RECORD(planner.name);
    setupWorld();
}


void runTest_ProbThresholdRRTConnect()
{
    std::cout << "prob threshold planner\n";
    bool use_prob_col = true;
    bool use_vox = false;
    VictorThresholdRRTConnect planner(&(sim_world->victor_model), use_vox, use_prob_col);
    planner.name = "Prob Threshold bi_rrt";
    runTest(planner);
}

void runTest_VoxThresholdRRTConnect()
{
    std::cout << "vox threshold planner\n";
    bool use_prob_col = false;
    bool use_vox = true;
    VictorThresholdRRTConnect planner(&(sim_world->victor_model), use_vox, use_prob_col);
    planner.name = "Vox Threshold bi_rrt";

    runTest(planner);
}

void runTest_ProbColCostRRTConnect()
{
    std::cout << "\n\nAnytime ProbCol test\n\n\n";
    VictorProbColCostRRTConnect planner(&(sim_world->victor_model));
    planner.name = "Prob Col Anytime";
    runTest(planner);
}

void runTest_VoxCostRRTConnect()
{
    std::cout << "\n\nAnytime MinVox test\n\n\n";
    VictorVoxCostRRTConnect planner(&(sim_world->victor_model));
    planner.name = "Vox Cost Anytime";
    runTest(planner);
}

void runTest_PlanUpProbColCostRRTConnect()
{
    std::cout << "\n\nPlanUp ProbCol test\n\n\n";
    VictorProbColCostRRTConnect planner(&(sim_world->victor_model));
    planner.use_anytime_planner = false;
    planner.name = "Prob Col Increasing";
    runTest(planner);
}

void runTest_PlanUpVoxCostRRTConnect()
{
    std::cout << "\n\nPlanUp MinVox test\n\n\n";
    VictorVoxCostRRTConnect planner(&(sim_world->victor_model));
    planner.use_anytime_planner = false;
    planner.name = "Vox Cost Increasing";
    runTest(planner);
}

void runTest_ProbRRTStar()
{
    std::cout << "\n\nRRT Star ProbCol test\n\n\n";
    bool use_prob_cost = true;
    VictorRRTStar planner(&(sim_world->victor_model), use_prob_cost);

    planner.name = "Prob Col RRTStar";
    runTest(planner);
}

void runTest_Diverse()
{
    std::cout << "\n\nDiverse test\n\n\n";
    DiversePlanner planner(&(sim_world->victor_model));
    planner.name = "DiversePlanner";
    runTest(planner);
}


int main(int argc, char* argv[])
{


    icl_core::logging::initialize(argc, argv);

    // sim_world = std::make_shared<SimWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    PROFILE_REINITIALIZE(20, 10000);

    setupWorld();

    int num_trials = 20;
    for(int i=0; i<num_trials; i++)
    {
        std::cout << "\n\n\n\n!!!!!!!!!!!!!!\nTrial " << i + 1<< " of " << num_trials << "!!!!!!!!!!!!!\n\n\n\n\n";

        // runTest_Diverse();
        // runTest_ProbThresholdRRTConnect();
        // runTest_VoxThresholdRRTConnect();
        // runTest_ProbColCostRRTConnect();
        // runTest_VoxCostRRTConnect();
        runTest_PlanUpProbColCostRRTConnect();
        // runTest_PlanUpVoxCostRRTConnect();
        // runTest_ProbRRTStar();
    }
    


    std::string filename = "ISER_sim_times_" + arc_helpers::GetCurrentTimeAsString();
    // PROFILE_WRITE_SUMMARY_FOR_ALL("victor_sim_times.txt");
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
    std::cout << "\n\n\n\n\n\nWrote summary!!\n\n\n\n\n\n";

    

    
    

    return 0;



    sim_world->gvl.reset();
    
}
