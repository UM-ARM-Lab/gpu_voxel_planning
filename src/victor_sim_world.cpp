
#include "victor_sim_world.hpp"
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <csignal>
#include <vector>
#include <cmath>





namespace ob = ompl::base;
namespace og = ompl::geometric;

#define VICTOR_WIDTH 0.15
#define NUM_SETS 100
#define EXPLORE_SLEEP_TIME 100000

std::shared_ptr<VictorWorld> victorWorld;

/***********************************************
 **                VICTOR WORLD                  **
 ***********************************************/

VictorWorld::VictorWorld():
    num_observed_sets(0)
{
    gvl = gpu_voxels::GpuVoxels::getInstance();



    gvl->initialize(100,100,100, 0.01);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_ACTUAL_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_QUERY_MAP);
    gvl->addMap(MT_PROBAB_VOXELMAP, VICTOR_SWEPT_VOLUME_MAP);
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
    // gvl->visualizeMap(VICTOR_ACTUAL_MAP);
    gvl->visualizeMap(OBSTACLES_ACTUAL_MAP);
}

VictorWorld::~VictorWorld()
{
    gvl.reset();
}


void insertVictorIntoMap(const VictorConfig &c, const std::string &map_name)
{
}

/*
 * Returns true if there is a collision 
 * if there is a collision && create_obstacle, then an obstacle will be added to the set
 */
bool VictorWorld::updateActual(const VictorConfig &c, bool create_obstacle)
{
    // std::cout << "Updating actual\n";
    gvl->clearMap(VICTOR_ACTUAL_MAP);
    gvl->insertVictorIntoMap(c, VICTOR_ACTUAL_MAP, PROB_OCCUPIED);
    gvl->visualizeMap(VICTOR_ACTUAL_MAP);
    for(int ind = 0; ind < num_observed_sets; ind++)
    {
        gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap(SEEN_OBSTACLE_SETS[ind]);
        voxelmap::ProbVoxelMap* obstacles = obstacles_ptr->as<voxelmap::ProbVoxelMap>();
        
        obstacles->subtract(gvl->getMap(VICTOR_SWEPT_VOLUME_MAP)->as<voxelmap::ProbVoxelMap>());
        gvl->visualizeMap(SEEN_OBSTACLE_SETS[ind]);
    }


    if(isActualCollision())
    {

        if(create_obstacle)
        {
            // std::cout << "Createing obstacle...";
            gvl->insertBoxIntoMap(b.lower(), b.upper(), SEEN_OBSTACLE_SETS[num_observed_sets], PROB_OCCUPIED);

            if(num_observed_sets >= NUM_SETS)
            {
                std::cout << "Reached set limit, not adding any more\n";
                return true;
            }

            // std::cout << "Done\n";
            num_observed_sets++;
        }


        return true;
    }
    
    gvl->insertBoxIntoMap(b.lower(), b.upper(), VICTOR_SWEPT_VOLUME_MAP, PROB_OCCUPIED);
    // std::cout << "finished updating actual\n";
    return false;
}

bool VictorWorld::isActualCollision()
{
    size_t num_colls = gvl->getMap(VICTOR_ACTUAL_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(OBSTACLES_ACTUAL_MAP)->as<voxelmap::ProbVoxelMap>());
    return num_colls > 0;
}


bool VictorWorld::querySeenState(const VictorConfig &b)
{
    resetQuery();
    addQueryState(b);
    return countSeenCollisionsInQuery() == 0;
}

// void querySeenPath();

void VictorWorld::addQueryState(const VictorConfig &b)
{
    gvl->insertVictorIntoMap(b.lower(), b.upper(), VICTOR_QUERY_MAP, PROB_OCCUPIED);
}

void VictorWorld::resetQuery()
{
    gvl->clearMap(VICTOR_QUERY_MAP);
}

size_t VictorWorld::countSeenCollisionsInQuery()
{
    size_t total_col = 0;
    std::vector<size_t> query_collisions = countSeenCollisionsInQueryForEach();
    for(auto cols: query_collisions)
    {
        total_col += cols;
    }
    return total_col;
}

std::vector<size_t> VictorWorld::countSeenCollisionsInQueryForEach()
{
    std::vector<size_t> query_collisions;
    query_collisions.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        query_collisions[i] = gvl->getMap(VICTOR_QUERY_MAP)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(SEEN_OBSTACLE_SETS[i])->as<voxelmap::ProbVoxelMap>());
    }
    return query_collisions;
}

size_t VictorWorld::getNumOccupiedVoxels(const std::string& map_name)
{
    return countIntersect(FULL_MAP, map_name);
}

size_t VictorWorld::countIntersect(const std::string& map_1, const std::string& map_2)
{
    return gvl->getMap(map_1)->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap(map_2)->as<voxelmap::ProbVoxelMap>());
}

/*
 *  Returns the number of voxels in each seen collision map
 */
std::vector<size_t> VictorWorld::seenSizes()
{
    std::vector<size_t> seen_sizes;
    seen_sizes.resize(num_observed_sets);
    for(int i=0; i < num_observed_sets; i++)
    {
        seen_sizes[i] = getNumOccupiedVoxels(SEEN_OBSTACLE_SETS[i]);
    }
    return seen_sizes;

}

void VictorWorld::initializeObstacles()
{
    std::vector<Box> boxes;
    double sl = 0.17;
    for(double x=0; x+sl < 1; x+=sl+VICTOR_WIDTH+0.05)
    {
        for(double z=0; z+sl<1; z+= sl+VICTOR_WIDTH+0.05)
        {
            boxes.emplace_back(x,0.5,z,sl);
        }
    }


    for(auto &box: boxes)
    {
        gvl->insertVictorIntoMap(box.lower(), box.upper(), OBSTACLES_ACTUAL_MAP, PROB_OCCUPIED);
    }
    gvl->visualizeMap(OBSTACLES_ACTUAL_MAP);
}


void VictorWorld::doVis()
{
    gvl->visualizeMap(VICTOR_ACTUAL_MAP, true);
    // std::cout << "Visualizing!!\n";
}



Victor VictorWorld::stateToVictor(const ompl::base::State *state) const
{
    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    VictorConfig jvm;
    for(size_t i=0; i<right_arm_joint_names.size(); i++)
    {
        jvm[right_arm_joint_names[i]] = values[i];
    }
    return jvm;
}





/***********************************************
 **               VALIDATOR                   **
 ***********************************************/
VictorValidator::VictorValidator(const ob::SpaceInformationPtr &si,
                           VictorWorld* victor_world):
    ob::StateValidityChecker(si),
    ob::MotionValidator(si)
{
    spi_ptr = si;
    victor_world_ptr = victor_world;
}

bool VictorValidator::isValid(const ob::State *state) const
{
    return victor_world_ptr->querySeenState(victor_world_ptr->stateToVictor(state));
}

bool VictorValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                               std::pair< ompl::base::State*, double > & lastValid) const
{
    std::cout << "Checking motion...";
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
    std::cout << "Done\n";

    return result;
}

bool VictorValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::pair< ompl::base::State*, double > lastValid;
    return checkMotion(s1, s2, lastValid);
}





/***********************************************
 **            PATH VALIDATOR                 **
 ***********************************************/
VictorPathValidator::VictorPathValidator(ob::SpaceInformationPtr si,
                                   VictorWorld* victor_world) :
    PathValidator(si)
{
    victor_world_ptr = victor_world;
}

void VictorPathValidator::setProbabilityThreshold(double th)
{
    threshold = th;
}

bool VictorPathValidator::checkPath(const std::vector<ompl::base::State*> path,
                                 size_t &collision_index)
{
    // std::cout << "Checking path...";

    
    ompl::base::StateSpace *stateSpace_ = si_->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_world_ptr->resetQuery();
    ob::State *test = si_->allocState();

    double prob_col = 0.0;
        
    for(collision_index = 0; collision_index < (path.size() - 1); collision_index ++)
    {
        const ob::State *s1 = path[collision_index];
        const ob::State *s2 = path[collision_index + 1];
        int nd = stateSpace_->validSegmentCount(s1, s2);

        for(int j = 0; j < nd; j++)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(test));
        }
        std::vector<size_t> seen_col_voxels = victor_world_ptr->countSeenCollisionsInQueryForEach();
        std::vector<size_t> seen_sizes = victor_world_ptr->seenSizes();
        std::vector<double> p_no_collision;
        p_no_collision.resize(seen_col_voxels.size());
        double p_no_col_seen = 1.0;

        for(size_t i=0; i < seen_sizes.size(); i++)
        {
            p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
            assert(p_no_collision[i] <= 1.0);
            p_no_col_seen *= p_no_collision[i];
        }

        size_t path_size = victor_world_ptr->countIntersect(FULL_MAP, VICTOR_QUERY_MAP);
        size_t total_size = victor_world_ptr->countIntersect(FULL_MAP, FULL_MAP);
        size_t known_free_size = victor_world_ptr->countIntersect(VICTOR_SWEPT_VOLUME_MAP, VICTOR_QUERY_MAP);

        double num_occupied = (double)(path_size - known_free_size);
        double frac_occupied = num_occupied / (double) total_size;
        
        double p_no_col_unseen = std::pow(1.0 - frac_occupied, 30);

        prob_col = 1.0 - p_no_col_seen * p_no_col_unseen;

        if (prob_col > 1.0)
        {
            std::cout << "Prob_col " << prob_col;
            std::cout << ", p_no_col_seen " << p_no_col_seen;
            std::cout << ", p_no_col_unseen " << p_no_col_unseen;
            // assert (prob_col <= 1.0);
        }

        
        if(prob_col > threshold)
        {
            break;
        }
    }
    si_->freeState(test);
    

    // std::cout << "Finished\n";
    // std::cout << "pathsize " << path.size();
    // std::cout << " col index " << collision_index << "\n";
    // std::cout << "Threshold " << threshold << "\n";
    return prob_col <= threshold;
}





/***********************************************
 **           Victor MinVox Objective            **
 ***********************************************/

VictorMinVoxObjective::VictorMinVoxObjective(ompl::base::SpaceInformationPtr si,
                                       VictorWorld* victor_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    victor_world_ptr = victor_world;
}

ob::Cost VictorMinVoxObjective::stateCost(const ob::State *state) const
{
    victor_world_ptr->resetQuery();
    victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(state));
    return ob::Cost(victor_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost VictorMinVoxObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << victor_world_ptr->countSeenCollisionsInQuery() << "\n";
    return ob::Cost(victor_world_ptr->countSeenCollisionsInQuery());
}



/***********************************************
 **       Victor Collision Prob Objective        **
 ***********************************************/

VictorMinColProbObjective::VictorMinColProbObjective(ompl::base::SpaceInformationPtr si,
                                               VictorWorld* victor_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    victor_world_ptr = victor_world;
}

ob::Cost VictorMinColProbObjective::stateCost(const ob::State *state) const
{
    victor_world_ptr->resetQuery();
    victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(state));
    return ob::Cost(victor_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost VictorMinColProbObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << victor_world_ptr->countSeenCollisionsInQuery() << "\n";
    std::vector<size_t> seen_col_voxels = victor_world_ptr->countSeenCollisionsInQueryForEach();
    std::vector<size_t> seen_sizes = victor_world_ptr->seenSizes();

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
 **  Victor Collision Prob With Swept Objective  **
 ***********************************************/

VictorMinColProbSweptObjective::VictorMinColProbSweptObjective(ompl::base::SpaceInformationPtr si,
                                               VictorWorld* victor_world) :
    ob::OptimizationObjective(si)
{
    spi_ptr = si;
    victor_world_ptr = victor_world;
    std::cout << "Creating optimizationobjective\n";
}

ob::Cost VictorMinColProbSweptObjective::stateCost(const ob::State *state) const
{
    std::cout << "State cost\n";
    victor_world_ptr->resetQuery();
    victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(state));
    return ob::Cost(victor_world_ptr->countSeenCollisionsInQuery());
}

ob::Cost VictorMinColProbSweptObjective::motionCost(const ob::State *s1,
                                        const ob::State *s2) const
{
    std::cout << "starting swept objective\n";
    ompl::base::StateSpace *stateSpace_ = spi_ptr->getStateSpace().get();
    assert(stateSpace_ != nullptr);

    victor_world_ptr->resetQuery();
    int nd = stateSpace_->validSegmentCount(s1, s2);
    ob::State *test = spi_ptr->allocState();
    for(int j = 0; j < nd; j++)
    {
        stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
        victor_world_ptr->addQueryState(victor_world_ptr->stateToVictor(test));
    }
        
    spi_ptr->freeState(test);
    // std::cout << "Motion cost is " << victor_world_ptr->countSeenCollisionsInQuery() << "\n";
    std::vector<size_t> seen_col_voxels = victor_world_ptr->countSeenCollisionsInQueryForEach();
    std::vector<size_t> seen_sizes = victor_world_ptr->seenSizes();

    std::vector<double> p_no_collision;
    p_no_collision.resize(seen_col_voxels.size());
    double p_no_col_seen = 1.0;

    for(size_t i=0; i < seen_sizes.size(); i++)
    {
        p_no_collision[i] = 1.0 - (double)seen_col_voxels[i] / (double)seen_sizes[i];
        assert(p_no_collision[i] <= 1.0);
        p_no_col_seen *= p_no_collision[i];
    }


    std::cout << "staring count intersection\n";
    size_t path_size = victor_world_ptr->countIntersect(FULL_MAP, VICTOR_QUERY_MAP);
    size_t total_size = victor_world_ptr->countIntersect(FULL_MAP, FULL_MAP);
    size_t known_free_size = victor_world_ptr->countIntersect(VICTOR_SWEPT_VOLUME_MAP, VICTOR_QUERY_MAP);

    double num_occupied = (double)(path_size - known_free_size);
    double frac_occupied = num_occupied / (double) total_size;
    
    double p_no_col_unseen = std::pow(1.0 - frac_occupied, 10);

    

    std::cout << "num_occupied:  " << num_occupied << "\n";
    std::cout << "p_no_col_unseen: " << p_no_col_unseen << "\n";
    std::cout << "Col Prob " << 1.0 - p_no_col_seen * p_no_col_unseen << "\n";
    std::cout << "num_observed sets " << victor_world_ptr->num_observed_sets << "\n";
    std::cout << "ending swept objective\n";

    
    // return ob::Cost(1.0 - p_no_col_seen);
    return ob::Cost(1.0 - p_no_col_seen * p_no_col_unseen);
}





/***********************************************
 **                Victor Planner                **
 ***********************************************/

VictorPlanner::VictorPlanner(VictorWorld* victor_world)
{
    victor_world_ptr = victor_world;

    space = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0.0);
    bounds.setHigh(1.0 - VICTOR_WIDTH);
    
    space->setBounds(bounds);
    spi_ptr = std::make_shared<ob::SpaceInformation>(space);
    v_ptr = std::make_shared<VictorValidator>(spi_ptr, victor_world);
    simp_ = std::make_shared<og::PathSimplifier>(spi_ptr);


}

/*
 *  Tried to probe around a few points around the victor
 */
void VictorPlanner::probeAround(const Victor &b, double delta)
{
    
    double x = b.x, y = b.y, z = b.z;
    std::cout << x << ", " << y << ", " << z << "\n";
    double sl = b.sl;
    victor_world_ptr->updateActual(Victor(x+delta,y,z,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
    victor_world_ptr->updateActual(Victor(x-delta,y,z,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
    victor_world_ptr->updateActual(Victor(x,y+delta,z,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
    victor_world_ptr->updateActual(Victor(x,y-delta,z,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
    victor_world_ptr->updateActual(Victor(x,y,z+delta,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
    victor_world_ptr->updateActual(Victor(x,y,z-delta,sl));
    victor_world_ptr->doVis();
    usleep(EXPLORE_SLEEP_TIME);
}


/*
 * Returns true if the path succeeded
 */
bool VictorPlanner::executePath(og::PathGeometric* path)
{

    size_t n = path->getStateCount();
    ob::StateSpace *stateSpace = spi_ptr->getStateSpace().get();
    std::cout << "Executing path...";

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
            
            Victor b = victor_world_ptr->stateToVictor(showState);
            if(victor_world_ptr->updateActual(b, true))
            {
                stateSpace->interpolate(s1, s2, (double)(j-1) / (double)nd, showState);
                probeAround(victor_world_ptr->stateToVictor(showState), 0.02);
                std::cout << "Added collision sets\n";
                return false;
            }
            victor_world_ptr->doVis();
            usleep(50000);

        }

        spi_ptr->freeState(showState);

    }
    std::cout << "Path success!\n";
    return true;
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::planPathDouble(std::vector<double> start,
                                                     std::vector<double> goal)
{
    ob::ScopedState<> start_ss(space);
    ob::ScopedState<> goal_ss(space);
    // std::cout << "start: ";
    for(size_t i=0; i<start.size(); i++)
    {
        // std::cout << start[i] << ", ";
        start_ss[i] = start[i];
        goal_ss[i] = goal[i];
    }
    // std::cout << "\n";
    // std::cout << "goal: ";
    for(size_t i=0; i<start.size(); i++)
    {
        // std::cout << goal[i] << ", ";
    }
    // std::cout << "\n";
    return planPath(start_ss, goal_ss);

}


Maybe::Maybe<ob::PathPtr> VictorPlanner::planPath(ompl::base::ScopedState<> start,
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


void VictorPlanner::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    planner->setProblemDefinition(pdef_);

}



/***********************************************
 **               Victor LBKPIECE                **
 ***********************************************/

VictorLBKPIECE::VictorLBKPIECE(VictorWorld* victor_world) :
    VictorPlanner(victor_world)
{
    initializePlanner();
}

void VictorLBKPIECE::initializePlanner()
{
    spi_ptr->setStateValidityChecker(v_ptr);
    spi_ptr->setMotionValidator(v_ptr);
    
    planner = std::make_shared<og::LBKPIECE1>(spi_ptr);
    planner->setup();
}

void VictorLBKPIECE::postPlan(ob::PathPtr path)
{
    simp_->simplifyMax(*(path->as<og::PathGeometric>()));
}




/***********************************************
 **               Victor LazyRRTF                **
 ***********************************************/

VictorLazyRRTF::VictorLazyRRTF(VictorWorld* victor_world) :
    VictorPlanner(victor_world)
{
    initializePlanner();
}

void VictorLazyRRTF::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    std::shared_ptr<og::LazyRRTF> lrrtf = std::make_shared<og::LazyRRTF>(spi_ptr);
    pv_ = std::make_shared<VictorPathValidator>(spi_ptr, victor_world_ptr);
    lrrtf->setPathValidator(pv_);
    planner = lrrtf;
    planner->setup();
    threshold = 0.5;
}

Maybe::Maybe<ob::PathPtr> VictorLazyRRTF::planPath(ompl::base::ScopedState<> start,
                                                ompl::base::ScopedState<> goal)
{
    preparePlanner(start, goal);
    ob::PathPtr path;
    int planning_time = 5;

    pv_->setProbabilityThreshold(threshold);
    ob::PlannerStatus solved = planner->solve(planning_time);

    while(!solved)
    {
        threshold += 0.1;
        std::cout << "threshold " << threshold << "\n";
        pv_->setProbabilityThreshold(threshold);
        solved = planner->solve(planning_time);
        if(threshold > 1.2){
            break;
        }
    }
    threshold -= 0.1;

    
    if (!solved)
    {
        std::cout << "No solution could be found" << std::endl;
        return Maybe::Maybe<ob::PathPtr>();
    }

    path = pdef_->getSolutionPath();
    postPlan(path);
    return Maybe::Maybe<ob::PathPtr>(path);
}


void VictorLazyRRTF::preparePlanner(ob::ScopedState<> start, ob::ScopedState<> goal)
{
    pdef_ = std::make_shared<ob::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    planner->setProblemDefinition(pdef_);

}




/***********************************************
 **               Victor RRTStar                 **
 ***********************************************/

template <class T>
VictorRRTstar<T>::VictorRRTstar(VictorWorld* victor_world) :
    VictorPlanner(victor_world)
{
    initializePlanner();
    objective_ptr = std::make_shared<T>(spi_ptr, victor_world);
    objective_ptr -> setCostThreshold(ob::Cost(0.01));
}

template <class T>
void VictorRRTstar<T>::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    
    // planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner->setup();
}

template <class T>
void VictorRRTstar<T>::preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
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
    victorWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}
void killhandler(int)
{
    std::cout << "Resetting\n";
    victorWorld->gvl.reset();
    exit(EXIT_SUCCESS);
}




int main(int argc, char* argv[])
{
    
    icl_core::logging::initialize(argc, argv);

    victorWorld = std::make_shared<VictorWorld>();
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);


    
    // victorWorld->updateActual(Victor(0,0,0,0.1));
    victorWorld->doVis();

    victorWorld->initializeObstacles();

    int unused;
    std::cout << "Waiting for user input to start...\n";
    std::cin >> unused;

    // VictorPlanner planner(victorWorld.get());
    VictorLBKPIECE planner_unused0(victorWorld.get());
    VictorRRTstar<VictorMinVoxObjective> planner_unused1(victorWorld.get());
    VictorRRTstar<VictorMinColProbObjective> planner_unused2(victorWorld.get());
    VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(victorWorld.get());
    VictorTRRT<VictorMinColProbSweptObjective> planner_unused4(victorWorld.get());

    // VictorRRTstar<VictorMinColProbSweptObjective> planner_unused3(victorWorld.get());
    // VictorTRRT<VictorMinColProbSweptObjective> planner(victorWorld.get());
    VictorLazyRRTF planner(victorWorld.get());

    std::vector<double> start = {0.5, 0.2, 0.5};
    std::vector<double> goal = {0.5, 0.8, 0.5};

    bool reached_goal = false;

    while(!reached_goal)
    {
        victorWorld->updateActual(Victor(start[0], start[1], start[2], VICTOR_WIDTH));
        Maybe::Maybe<ob::PathPtr> path = planner.planPathDouble(start, goal);
        if(!path.Valid())
        {
            std::cout << "Path planning failed\n";
            break;
        }
        reached_goal = planner.executePath(path.Get()->as<og::PathGeometric>());
    }

    if(reached_goal)
        std::cout << "REACHED GOAL!\n";

    victorWorld->gvl.reset();
    
    // while(true)
    // {
    //     victorWorld.doVis();
    //     usleep(10000);
    // }
}
