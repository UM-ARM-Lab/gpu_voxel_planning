#include "box_world.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <vector>



namespace ob = ompl::base;
namespace og = ompl::geometric;

#define BOX_WIDTH 0.15



/***********************************************
 **                BOX WORLD                  **
 ***********************************************/

BoxWorld::BoxWorld()
{
    gvl = gpu_voxels::GpuVoxels::getInstance();

    gvl->initialize(100,100,100, 0.01);
    gvl->addMap(MT_PROBAB_VOXELMAP, BOX_ACTUAL_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, BOX_QUERY_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, OBSTACLES_ACTUAL_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, OBSTACLES_SEEN_MAP);
}


void BoxWorld::updateActual(const Box &b)
{
    gvl->clearMap(BOX_ACTUAL_MAP);
    gvl->insertBoxIntoMap(b.lower(), b.upper(), BOX_ACTUAL_MAP, PROB_OCCUPIED);
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
    
    size_t num_colls_pc = gvl->getMap(BOX_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(OBSTACLES_SEEN_MAP)->as<voxelmap::ProbVoxelMap>());

    return num_colls_pc;
}

void BoxWorld::initializeObstacles()
{
    std::vector<Box> boxes;
    double sl = 0.2;
    for(double x=0; x+sl < 1; x+=0.4)
    {
        for(double z=0; z+sl<1; z+= 0.4)
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
    std::cout << "Visualizing!!\n";
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
    si_ptr = si;
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
    ob::StateSpace *stateSpace = si_ptr->getStateSpace().get();
    int nd = stateSpace->validSegmentCount(s1, s2);

    ob::State *test = si_ptr->allocState();
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
    si_ptr->freeState(test);


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
    si_ptr = std::make_shared<ob::SpaceInformation>(space);
    v_ptr = std::make_shared<BoxValidator>(si_ptr, box_world);
    simp_ = std::make_shared<og::PathSimplifier>(si_ptr);

    si_ptr->setStateValidityChecker(v_ptr);
    si_ptr->setMotionValidator(v_ptr);
    
    planner = std::make_shared<og::LBKPIECE1>(si_ptr);
    planner->setup();

}

bool BoxPlanner::executePath(og::PathGeometric* path)
{

    size_t n = path->getStateCount();
    ob::StateSpace *stateSpace = si_ptr->getStateSpace().get();
    std::cout << "Executing path\n";

    for(size_t step = 0; step < n-1; step++)
    {
        ob::State *s1 = path->getState(step);
        ob::State *s2 = path->getState(step + 1);
        int nd = stateSpace->validSegmentCount(s1, s2);



        ob::State *showState = si_ptr->allocState();
        for(int j = 1; j<nd; j++)
        {

            std::cout << "Executing " << step << ", " << j << "\n";

            stateSpace->interpolate(s1, s2, (double)j / (double)nd, showState);
            
            Box b = box_world_ptr->stateToBox(showState);
            box_world_ptr->updateActual(b);
            box_world_ptr->doVis();
            // for(int i=0; i<100; i++)
            // {

            //     usleep(1000);
            // }
            usleep(100000);

        }

        si_ptr->freeState(showState);

    }
}

ob::PathPtr BoxPlanner::planPath(std::vector<double> start,
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

ob::PathPtr BoxPlanner::planPath(ompl::base::ScopedState<> start,
                                 ompl::base::ScopedState<> goal)
{
    preparePlanner(start, goal);
    ob::PathPtr path;
    ob::PlannerStatus solved = planner->solve(3);
    
    
    if (solved)
    {
        path = pdef_->getSolutionPath();
        simp_->simplifyMax(*(path->as<og::PathGeometric>()));
        return path;
    }
    else
    {
        std::cout << "No solution could be found" << std::endl;
        //this will probably cause a segfault?
        return path;
    }
}


void BoxPlanner::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    planner->setProblemDefinition(pdef_);

}







/***********************************************
 **                   MAIN                    **
 ***********************************************/


int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);

    BoxWorld boxWorld;
    boxWorld.initializeObstacles();
    boxWorld.updateActual(Box(0,0,0,0.1));

    int unused;
    std::cin >> unused;

    BoxPlanner planner(&boxWorld);

    std::vector<double> start = {0.2, 0.2, 0.2};
    std::vector<double> goal = {0.2, 0.8, 0.2};

    ob::PathPtr path= planner.planPath(start, goal);
    planner.executePath(path->as<og::PathGeometric>());

    
    while(true)
    {
        boxWorld.doVis();
        usleep(10000);
    }
}
