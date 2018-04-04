
#include "box_world.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <csignal>
#include <vector>




namespace ob = ompl::base;
namespace og = ompl::geometric;

#define BOX_WIDTH 0.15
#define NUM_SETS 100


std::shared_ptr<BoxWorld> boxWorld;

/***********************************************
 **                BOX WORLD                  **
 ***********************************************/

BoxWorld::BoxWorld():
    num_observed_sets(0)
{
    gvl = gpu_voxels::GpuVoxels::getInstance();



    gvl->initialize(100,100,100, 0.01);
    gvl->addMap(MT_PROBAB_VOXELMAP, BOX_ACTUAL_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, BOX_QUERY_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, BOX_SWEPT_VOLUME_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, OBSTACLES_ACTUAL_MAP);
    // gvl->addMap(MT_PROBAB_VOXELMAP, OBSTACLES_SEEN_MAP);

    SEEN_OBSTACLE_SETS.resize(NUM_SETS);
    for(int i=0; i < NUM_SETS; i++)
    {
        SEEN_OBSTACLE_SETS[i] = "seen_obstacles_" + std::to_string(i);
        gvl->addMap(MT_PROBAB_VOXELMAP, SEEN_OBSTACLE_SETS[i]);
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[i]);
    }

    // gvl->visualizeMap(OBSTACLES_SEEN_MAP);
    // gvl->visualizeMap(BOX_ACTUAL_MAP);
    gvl->visualizeMap(OBSTACLES_ACTUAL_MAP);
}

BoxWorld::~BoxWorld()
{
    gvl.reset();
}


/*
 * Returns true if there is a collision 
 */
bool BoxWorld::updateActual(const Box &b)
{
    gvl->clearMap(BOX_ACTUAL_MAP);
    gvl->insertBoxIntoMap(b.lower(), b.upper(), BOX_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->visualizeMap(BOX_ACTUAL_MAP);
    if(isActualCollision())
    {
        int ind = num_observed_sets;
        gvl->insertBoxIntoMap(b.lower(), b.upper(), SEEN_OBSTACLE_SETS[ind], PROB_OCCUPIED);
        gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(SEEN_OBSTACLE_SETS[ind]);
        voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
        
        obstacles->subtract(gvl->getMap(BOX_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());

        gvl->visualizeMap(SEEN_OBSTACLE_SETS[ind]);
        num_observed_sets++;
        return true;
    }
    
    gvl->insertBoxIntoMap(b.lower(), b.upper(), BOX_SWEPT_VOLUME_MAP, PROB_OCCUPIED);
    return false;
}

bool BoxWorld::isActualCollision()
{
    size_t num_colls = gvl->getMap(BOX_ACTUAL_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(OBSTACLES_ACTUAL_MAP)->as<voxelmap::ProbVoxelMap>());
    return num_colls > 0;
}


bool BoxWorld::querySeenState(const Box &b)
{
    resetQuery();
    addQueryState(b);
    return countSeenCollisionsInQuery() == 0;
}

// void querySeenPath();

void BoxWorld::addQueryState(const Box &b)
{
    gvl->insertBoxIntoMap(b.lower(), b.upper(), BOX_QUERY_MAP, PROB_OCCUPIED);
}

void BoxWorld::resetQuery()
{
    gvl->clearMap(BOX_QUERY_MAP);
}

size_t BoxWorld::countSeenCollisionsInQuery()
{
    size_t total_col = 0;
    for(int i=0; i < num_observed_sets; i++)
    {
        size_t num_colls_pc = gvl->getMap(BOX_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(SEEN_OBSTACLE_SETS[i])->as<voxelmap::ProbVoxelMap>());
        total_col += num_colls_pc;
    }

    return total_col;
}

void BoxWorld::initializeObstacles()
{
    std::vector<Box> boxes;
    double sl = 0.17;
    for(double x=0; x+sl < 1; x+=sl+BOX_WIDTH+0.05)
    {
        for(double z=0; z+sl<1; z+= sl+BOX_WIDTH+0.05)
        {
            boxes.emplace_back(x,0.5,z,sl);
        }
    }


    for(auto &box: boxes)
    {
        gvl->insertBoxIntoMap(box.lower(), box.upper(), OBSTACLES_ACTUAL_MAP, PROB_OCCUPIED);
    }
    gvl->visualizeMap(OBSTACLES_ACTUAL_MAP);
}


void BoxWorld::doVis()
{
    gvl->visualizeMap(BOX_ACTUAL_MAP, true);
    // std::cout << "Visualizing!!\n";
}



Box BoxWorld::stateToBox(const ompl::base::State *state) const
{
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    return Box(values[0], values[1], values[2], BOX_WIDTH);
}





/***********************************************
 **               VALIDATOR                   **
 ***********************************************/
BoxValidator::BoxValidator(const ob::SpaceInformationPtr &si,
                           BoxWorld* box_world):
    ob::StateValidityChecker(si),
    ob::MotionValidator(si)
{
    spi_ptr = si;
    box_world_ptr = box_world;
}

bool BoxValidator::isValid(const ob::State *state) const
{
    return box_world_ptr->querySeenState(box_world_ptr->stateToBox(state));
}

bool BoxValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                               std::pair< ompl::base::State*, double > & lastValid) const
{
    bool result = true;
    ob::StateSpace *stateSpace = spi_ptr->getStateSpace().get();
    int nd = stateSpace->validSegmentCount(s1, s2);

    ob::State *test = spi_ptr->allocState();
    for(int j = 1; j<=nd; j++)
    {
        stateSpace->interpolate(s1, s2, (double)j / (double)nd, test);
        if (!isValid(test))
        {
            lastValid.second = (double)(j-1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
            break;
        }
    }
    spi_ptr->freeState(test);


    if(result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool BoxValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::pair< ompl::base::State*, double > lastValid;
    return checkMotion(s1, s2, lastValid);
}





/***********************************************
 **                Box Planner                **
 ***********************************************/

BoxPlanner::BoxPlanner(BoxWorld* box_world)
{
    box_world_ptr = box_world;

    space = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0.0);
    bounds.setHigh(1.0 - BOX_WIDTH);
    
    space->setBounds(bounds);
    spi_ptr = std::make_shared<ob::SpaceInformation>(space);
    v_ptr = std::make_shared<BoxValidator>(spi_ptr, box_world);
    simp_ = std::make_shared<og::PathSimplifier>(spi_ptr);

    spi_ptr->setStateValidityChecker(v_ptr);
    spi_ptr->setMotionValidator(v_ptr);
    
    planner = std::make_shared<og::LBKPIECE1>(spi_ptr);
    planner->setup();

}

/*
 * Returns true if the path succeeded
 */
bool BoxPlanner::executePath(og::PathGeometric* path)
{

    size_t n = path->getStateCount();
    ob::StateSpace *stateSpace = spi_ptr->getStateSpace().get();
    std::cout << "Executing path\n";

    for(size_t step = 0; step < n-1; step++)
    {
        ob::State *s1 = path->getState(step);
        ob::State *s2 = path->getState(step + 1);
        int nd = stateSpace->validSegmentCount(s1, s2);



        ob::State *showState = spi_ptr->allocState();
        for(int j = 1; j<nd; j++)
        {

            std::cout << "Executing " << step << ", " << j << "\n";

            stateSpace->interpolate(s1, s2, (double)j / (double)nd, showState);
            
            Box b = box_world_ptr->stateToBox(showState);
            if(box_world_ptr->updateActual(b))
            {
                return false;
            }
            box_world_ptr->doVis();
            usleep(100000);

        }

        spi_ptr->freeState(showState);

    }
    return true;
}

Maybe::Maybe<ob::PathPtr> BoxPlanner::planPath(std::vector<double> start,
                                               std::vector<double> goal)
{
    ob::ScopedState<> start_ss(space);
    ob::ScopedState<> goal_ss(space);
    std::cout << "start: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << start[i] << ", ";
        start_ss[i] = start[i];
        goal_ss[i] = goal[i];
    }
    std::cout << "\n";
    std::cout << "goal: ";
    for(size_t i=0; i<start.size(); i++)
    {
        std::cout << goal[i] << ", ";
    }
    std::cout << "\n";
    return planPath(start_ss, goal_ss);

}


Maybe::Maybe<ob::PathPtr> BoxPlanner::planPath(ompl::base::ScopedState<> start,
                                               ompl::base::ScopedState<> goal)
{
    preparePlanner(start, goal);
    ob::PathPtr path;
    ob::PlannerStatus solved = planner->solve(3);
    
    
    if (solved)
    {
        path = pdef_->getSolutionPath();
        simp_->simplifyMax(*(path->as<og::PathGeometric>()));
        return Maybe::Maybe<ob::PathPtr>(path);
    }
    else
    {
        std::cout << "No solution could be found" << std::endl;
        //this will probably cause a segfault?
        return Maybe::Maybe<ob::PathPtr>();
    }
}


void BoxPlanner::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    planner->setProblemDefinition(pdef_);

}







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


int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    boxWorld = std::make_shared<BoxWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    
    // boxWorld->updateActual(Box(0,0,0,0.1));
    boxWorld->doVis();

    boxWorld->initializeObstacles();

    int unused;
    std::cin >> unused;

    BoxPlanner planner(boxWorld.get());

    std::vector<double> start = {0.5, 0.2, 0.5};
    std::vector<double> goal = {0.5, 0.8, 0.5};

    bool reached_goal = false;

    while(!reached_goal)
    {
        boxWorld->updateActual(Box(start[0], start[1], start[2], BOX_WIDTH));
        Maybe::Maybe<ob::PathPtr> path = planner.planPath(start, goal);
        if(!path.Valid())
        {
            std::cout << "Path planning failed\n";
            break;
        }
        reached_goal = planner.executePath(path.Get()->as<og::PathGeometric>());
    }

    if(reached_goal)
        std::cout << "REACHED GOAL!\n";

    
    // while(true)
    // {
    //     boxWorld.doVis();
    //     usleep(10000);
    // }
}
