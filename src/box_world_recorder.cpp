#include "box_world.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <csignal>
#include <vector>

#define ENABLE_PROFILING
#include "arc_utilities/timing.hpp"

#define BOX_WIDTH 0.15

namespace ob = ompl::base;
namespace og = ompl::geometric;

const std::string LBKPIECE_TIME = "lbkpiece";
const std::string RRTSTAR_MINVOX_TIME = "RRTstar(minvox)";
const std::string RRTSTAR_MINPROB_TIME = "RRTstar(minprob)";
const std::string RRTSTAR_MINPROB_SWEPT_TIME = "RRTstar(sweptprob)";
const std::string TRRT_MINPROB_SWEPT_TIME = "TRRT(sweptprob)";
const std::string ALL_COUNTS = "All counts";


std::shared_ptr<BoxWorld> boxWorld;

/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int)
{
    std::cout << "Resetting\n";
    boxWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    boxWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}



bool plan(BoxPlanner &planner, const std::string &name)
{
    std::vector<double> start = {0.5, 0.2, 0.5};
    std::vector<double> goal = {0.5, 0.8, 0.5};

    bool reached_goal = false;

    while(!reached_goal)
    {
        boxWorld->updateActual(Box(start[0], start[1], start[2], BOX_WIDTH));
        PROFILE_START(name + " plan");
        Maybe::Maybe<ob::PathPtr> path = planner.planPath(start, goal);
        PROFILE_RECORD(name + " plan");
        if(!path.Valid())
        {
            std::cout << "Path planning failed\n";
            break;
        }
        PROFILE_START(name + " execute");
        reached_goal = planner.executePath(path.Get()->as<og::PathGeometric>());
        PROFILE_RECORD(name + " execute");
    }

    if(reached_goal)
        std::cout << "REACHED GOAL!\n";
    return reached_goal;
}

void setupWorld()
{
    boxWorld = std::make_shared<BoxWorld>();
    boxWorld->doVis();
    boxWorld->initializeObstacles();
}

template <class T>
bool testPlanner(std::string logger)
{
    std::cout << "Starting " << logger << "\n";
    setupWorld();
    PROFILE_START(logger);
    T planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;

}


bool testLBKPIECE()
{
    std::string logger = LBKPIECE_TIME;
    std::cout << "Starting " << logger << "\n";
    setupWorld();
    PROFILE_START(logger);
    BoxLBKPIECE planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;
}

bool testMinVoxRRTstar()
{
    std::string logger = RRTSTAR_MINVOX_TIME;
    std::cout << "Starting " << logger << "\n";
    setupWorld();
    PROFILE_START(logger);
    BoxRRTstar<BoxMinVoxObjective> planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;
}

bool testMinColProbRRTstar()
{
    std::string logger = RRTSTAR_MINPROB_TIME;
    std::cout << "Starting " << logger << "\n";

    setupWorld();
    PROFILE_START(logger);
    BoxRRTstar<BoxMinColProbObjective> planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;
}

bool testMinColProbSweptRRTstar()
{
    std::string logger = RRTSTAR_MINPROB_SWEPT_TIME;
    std::cout << "Starting " << logger << "\n";
    setupWorld();
    PROFILE_START(logger);
    BoxRRTstar<BoxMinColProbSweptObjective> planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;
}

bool testMinColProbSweptTRRT()
{
    std::string logger = TRRT_MINPROB_SWEPT_TIME;
    std::cout << "Starting " << logger << "\n";

    setupWorld();
    PROFILE_START(logger);
    BoxTRRT<BoxMinColProbSweptObjective> planner(boxWorld.get());
    bool success = plan(planner, logger);
    boxWorld->gvl.reset();
    if(success)
        PROFILE_RECORD(logger);
    return success;
}


int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);


    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    PROFILE_REINITIALIZE(20, 10000);
    
    // boxWorld->updateActual(Box(0,0,0,0.1));

    // BoxLBKPIECE planner1(boxWorld.get());
    // BoxRRTstar<BoxMinVoxObjective> planner2(boxWorld.get());
    // BoxRRTstar<BoxMinColProbObjective> planner3(boxWorld.get());

    int num_tests = 1;
    std::cout << "Running " << num_tests << " trials\n";

    for(int i=0; i<num_tests; i++)
    {
        PROFILE_START(ALL_COUNTS);
        PROFILE_RECORD(ALL_COUNTS);
        // testLBKPIECE();
        // testMinVoxRRTstar();
        testMinColProbRRTstar();
        testMinColProbSweptRRTstar();
        // testMinColProbSweptTRRT();
    }

    std::cout << "Recording\n";
    // int unused;
    // std::cin >> unused;
    
    std::vector<std::string> names = {
        ALL_COUNTS,
        LBKPIECE_TIME,
        LBKPIECE_TIME + " plan",
        LBKPIECE_TIME + " execute",
        RRTSTAR_MINVOX_TIME,
        RRTSTAR_MINVOX_TIME + " plan",
        RRTSTAR_MINVOX_TIME + " execute",
        RRTSTAR_MINPROB_TIME,
        RRTSTAR_MINPROB_TIME + " plan",
        RRTSTAR_MINPROB_TIME + " execute",
        RRTSTAR_MINPROB_SWEPT_TIME,
        RRTSTAR_MINPROB_SWEPT_TIME + " plan",
        RRTSTAR_MINPROB_SWEPT_TIME + " execute",
        TRRT_MINPROB_SWEPT_TIME,
        TRRT_MINPROB_SWEPT_TIME + " plan",
        TRRT_MINPROB_SWEPT_TIME + " execute",
    };
    PROFILE_WRITE_SUMMARY_FOR_GROUP("test.txt", names);

    std::cout << "writing finished\n";

    boxWorld->gvl.reset();
    return 0;
    // while(true)
    // {
    //     boxWorld.doVis();
    //     usleep(10000);
    // }
}

