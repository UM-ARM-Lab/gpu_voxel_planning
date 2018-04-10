
#include "box_world.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <csignal>
#include <vector>
#include <cmath>




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
    gvl->addMap(MT_PROBAB_VOXELMAP, FULL_MAP);

    gvl->insertBoxIntoMap(Vector3f(0,0,0), Vector3f(1,1,1), FULL_MAP, PROB_OCCUPIED);
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
 * if there is a collision && create_obstacle, then an obstacle will be added to the set
 */
bool BoxWorld::updateActual(const Box &b, bool create_obstacle)
{
    gvl->clearMap(BOX_ACTUAL_MAP);
    gvl->insertBoxIntoMap(b.lower(), b.upper(), BOX_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->visualizeMap(BOX_ACTUAL_MAP);
    for(int ind = 0; ind < num_observed_sets; ind++)
    {
        gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(SEEN_OBSTACLE_SETS[ind]);
        voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
        
        obstacles->subtract(gvl->getMap(BOX_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[ind]);
    }


    if(isActualCollision())
    {

        if(create_obstacle)
        {
            gvl->insertBoxIntoMap(b.lower(), b.upper(), SEEN_OBSTACLE_SETS[num_observed_sets], PROB_OCCUPIED);

            if(num_observed_sets > NUM_SETS)
            {
                std::cout << "Reached set limit, not adding any more\n";
                return true;
            }

            num_observed_sets++;
        }


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
    std::vector<size_t> query_collisions = countSeenCollisionsInQueryForEach();
    for(auto cols: query_collisions)
    {
        total_col += cols;
    }
    return total_col;
}

std::vector<size_t> BoxWorld::countSeenCollisionsInQueryForEach()
{
    std::vector<size_t> query_collisions;
    query_collisions.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        query_collisions[i] = gvl->getMap(BOX_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(SEEN_OBSTACLE_SETS[i])->as<voxelmap::ProbVoxelMap>());
    }
    return query_collisions;
}

size_t BoxWorld::getNumOccupiedVoxels(const std::string& map_name)
{
    return countIntersect(FULL_MAP, map_name);
}

size_t BoxWorld::countIntersect(const std::string& map_1, const std::string& map_2)
{
    return gvl->getMap(map_1)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(map_2)->as<voxelmap::ProbVoxelMap>());
}

/*
 *  Returns the number of voxels in each seen collision map
 */
std::vector<size_t> BoxWorld::seenSizes()
{
    std::vector<size_t> seen_sizes;
    seen_sizes.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        seen_sizes[i] = getNumOccupiedVoxels(SEEN_OBSTACLE_SETS[i]);
    }
    return seen_sizes;

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
 **           Box MinVox Objective            **
 ***********************************************/

BoxMinVoxObjective::BoxMinVoxObjective(ompl::base::SpaceInformationPtr si,
                                       BoxWorld* box_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    box_world_ptr = box_world;
}

ob::Cost BoxMinVoxObjective::stateCost(const ob::State *state) const
{
    box_world_ptr->resetQuery();
    box_world_ptr->addQueryState(box_world_ptr->stateToBox(state));
    return ob::Cost(box_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost BoxMinVoxObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    box_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        box_world_ptr->addQueryState(box_world_ptr->stateToBox(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << box_world_ptr->countSeenCollisionsInQuery() << "\n";
    return ob::Cost(box_world_ptr->countSeenCollisionsInQuery());
}



/***********************************************
 **       Box Collision Prob Objective        **
 ***********************************************/

BoxMinColProbObjective::BoxMinColProbObjective(ompl::base::SpaceInformationPtr si,
                                               BoxWorld* box_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    box_world_ptr = box_world;
}

ob::Cost BoxMinColProbObjective::stateCost(const ob::State *state) const
{
    box_world_ptr->resetQuery();
    box_world_ptr->addQueryState(box_world_ptr->stateToBox(state));
    return ob::Cost(box_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost BoxMinColProbObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    box_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        box_world_ptr->addQueryState(box_world_ptr->stateToBox(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << box_world_ptr->countSeenCollisionsInQuery() << "\n";
    std::vector<size_t> seen_col_voxels = box_world_ptr->countSeenCollisionsInQueryForEach();
    std::vector<size_t> seen_sizes = box_world_ptr->seenSizes();

    std::vector<double> p_no_collision;
    p_no_collision.resize(seen_col_voxels.size());
    double p_no_col = 1.0;

    for(size_t i=0; i < seen_sizes.size(); i++)
    {
        p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
        assert(p_no_collision[i] <= 1.0);
        p_no_col *= p_no_collision[i];
    }

    // std::cout << "Col Prob " << 1.0 - p_no_col << "\n";
    return ob::Cost(1.0-p_no_col);
}


/***********************************************
 **  Box Collision Prob With Swept Objective  **
 ***********************************************/

BoxMinColProbSweptObjective::BoxMinColProbSweptObjective(ompl::base::SpaceInformationPtr si,
                                               BoxWorld* box_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    box_world_ptr = box_world;
}

ob::Cost BoxMinColProbSweptObjective::stateCost(const ob::State *state) const
{
    box_world_ptr->resetQuery();
    box_world_ptr->addQueryState(box_world_ptr->stateToBox(state));
    return ob::Cost(box_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost BoxMinColProbSweptObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    box_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        box_world_ptr->addQueryState(box_world_ptr->stateToBox(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << box_world_ptr->countSeenCollisionsInQuery() << "\n";
    std::vector<size_t> seen_col_voxels = box_world_ptr->countSeenCollisionsInQueryForEach();
    std::vector<size_t> seen_sizes = box_world_ptr->seenSizes();

    std::vector<double> p_no_collision;
    p_no_collision.resize(seen_col_voxels.size());
    double p_no_col_seen = 1.0;

    for(size_t i=0; i < seen_sizes.size(); i++)
    {
        p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
        assert(p_no_collision[i] <= 1.0);
        p_no_col_seen *= p_no_collision[i];
    }


    size_t path_size = box_world_ptr->countIntersect(FULL_MAP, BOX_QUERY_MAP);
    size_t total_size = box_world_ptr->countIntersect(FULL_MAP, FULL_MAP);
    size_t known_free_size = box_world_ptr->countIntersect(BOX_SWEPT_VOLUME_MAP, BOX_QUERY_MAP);

    double num_occupied = (double)(path_size - known_free_size);
    double frac_occupied = num_occupied / (double) total_size;
    
    double p_no_col_unseen = std::pow(1.0 - frac_occupied, 30);

    

    // std::cout << "num_occupied:  " << num_occupied << "\n";
    // std::cout << "p_no_col_unseen: " << p_no_col_unseen << "\n";
    // std::cout << "Col Prob " << 1.0 - p_no_col_seen * p_no_col_unseen << "\n";
    // return ob::Cost(1.0 - p_no_col_seen * p_no_col_unseen);
    return ob::Cost(1.0 - p_no_col_seen * p_no_col_unseen);
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


}

/*
 *  Tried to probe around a few points around the box
 */
void BoxPlanner::probeAround(const Box &b, double delta)
{
    double x = b.x, y = b.y, z = b.z;
    std::cout << x << ", " << y << ", " << z << "\n";
    double sl = b.sl;
    box_world_ptr->updateActual(Box(x+delta,y,z,sl));
    box_world_ptr->doVis();
    usleep(200000);
    box_world_ptr->updateActual(Box(x-delta,y,z,sl));
    box_world_ptr->doVis();
    usleep(200000);
    box_world_ptr->updateActual(Box(x,y+delta,z,sl));
    box_world_ptr->doVis();
    usleep(200000);
    box_world_ptr->updateActual(Box(x,y-delta,z,sl));
    box_world_ptr->doVis();
    usleep(200000);
    box_world_ptr->updateActual(Box(x,y,z+delta,sl));
    box_world_ptr->doVis();
    usleep(200000);
    box_world_ptr->updateActual(Box(x,y,z-delta,sl));
    box_world_ptr->doVis();
    usleep(200000);
}


/*
 * Returns true if the path succeeded
 */
bool BoxPlanner::executePath(og::PathGeometric* path)
{

    size_t n = path->getStateCount();
    ob::StateSpace *stateSpace = spi_ptr->getStateSpace().get();
    // std::cout << "Executing path\n";

    for(size_t step = 0; step < n-1; step++)
    {
        ob::State *s1 = path->getState(step);
        ob::State *s2 = path->getState(step + 1);
        int nd = stateSpace->validSegmentCount(s1, s2);



        ob::State *showState = spi_ptr->allocState();
        for(int j = 1; j<nd; j++)
        {

            // std::cout << "Executing " << step << ", " << j << "\n";

            stateSpace->interpolate(s1, s2, (double)j / (double)nd, showState);
            
            Box b = box_world_ptr->stateToBox(showState);
            if(box_world_ptr->updateActual(b, false))
            {
                stateSpace->interpolate(s1, s2, (double)(j-1) / (double)nd, showState);
                probeAround(box_world_ptr->stateToBox(showState), 0.02);
                return false;
            }
            box_world_ptr->doVis();
            usleep(50000);

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
    int planning_time = 10;
    ob::PlannerStatus solved = planner->solve(planning_time);

    while(solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        planning_time *= 2;

        if(planning_time > 60*10)
        {
            "Failed to find a solution in 10 minutes. Giving up\n";
            return Maybe::Maybe<ob::PathPtr>();
        }
        
        std::cout << "Approximate solution, replanning for " << planning_time << " seconds\n";
        solved = planner->solve(planning_time);
    }
    
    if (!solved)
    {
        std::cout << "No solution could be found" << std::endl;
        return Maybe::Maybe<ob::PathPtr>();
    }

    path = pdef_->getSolutionPath();
    postPlan(path);
    return Maybe::Maybe<ob::PathPtr>(path);
}


void BoxPlanner::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    planner->setProblemDefinition(pdef_);

}



/***********************************************
 **               Box LBKPIECE                **
 ***********************************************/

BoxLBKPIECE::BoxLBKPIECE(BoxWorld* box_world) :
    BoxPlanner(box_world)
{
    initializePlanner();
}

void BoxLBKPIECE::initializePlanner()
{
    spi_ptr->setStateValidityChecker(v_ptr);
    spi_ptr->setMotionValidator(v_ptr);
    
    planner = std::make_shared<og::LBKPIECE1>(spi_ptr);
    planner->setup();
}

void BoxLBKPIECE::postPlan(ob::PathPtr path)
{
    simp_->simplifyMax(*(path->as<og::PathGeometric>()));
}



/***********************************************
 **               Box RRTStar                 **
 ***********************************************/

template <class T>
BoxRRTstar<T>::BoxRRTstar(BoxWorld* box_world) :
    BoxPlanner(box_world)
{
    initializePlanner();
    objective_ptr = std::make_shared<T>(spi_ptr, box_world);
    objective_ptr -> setCostThreshold(ob::Cost(0.01));
}

template <class T>
void BoxRRTstar<T>::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    
    // planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner->setup();
}

template <class T>
void BoxRRTstar<T>::preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(objective_ptr);
    planner->setProblemDefinition(pdef_);

}



/***********************************************
 **               Box TRRT                    **
 ***********************************************/

template <class T>
BoxTRRT<T>::BoxTRRT(BoxWorld* box_world) :
    BoxPlanner(box_world)
{
    initializePlanner();
    objective_ptr = std::make_shared<T>(spi_ptr, box_world);
    objective_ptr -> setCostThreshold(ob::Cost(0.01));
}

template <class T>
void BoxTRRT<T>::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    
    // planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner = std::make_shared<og::TRRT>(spi_ptr);
    planner->setup();
}

template <class T>
void BoxTRRT<T>::preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(objective_ptr);
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

    // BoxPlanner planner(boxWorld.get());
    BoxLBKPIECE planner_unused0(boxWorld.get());
    BoxRRTstar<BoxMinVoxObjective> planner_unused1(boxWorld.get());
    BoxRRTstar<BoxMinColProbObjective> planner_unused2(boxWorld.get());
    BoxRRTstar<BoxMinColProbSweptObjective> planner_unused3(boxWorld.get());
    BoxTRRT<BoxMinColProbSweptObjective> planner_unused4(boxWorld.get());

    BoxRRTstar<BoxMinColProbSweptObjective> planner(boxWorld.get());

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

    boxWorld->gvl.reset();
    
    // while(true)
    // {
    //     boxWorld.doVis();
    //     usleep(10000);
    // }
}
