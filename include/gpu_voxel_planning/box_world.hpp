#ifndef BOX_WORLD_HPP
#define BOX_WORLD_HPP


#include <gpu_voxels/GpuVoxels.h>

#include "path_validator.h"
#include "lazyrrt_fullpath.h"


#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/TRRT.h>

#include <arc_utilities/maybe.hpp>

#include <string>



const std::string BOX_ACTUAL_MAP = "box_actual";
const std::string BOX_QUERY_MAP = "box_query";
const std::string BOX_SWEPT_VOLUME_MAP = "box_swept_volume";
const std::string OBSTACLES_ACTUAL_MAP = "actual_obstacles";
const std::string FULL_MAP = "full_map";
// const std::string OBSTACLES_SEEN_MAP = "seen_obstacles";

std::vector<std::string> SEEN_OBSTACLE_SETS;


#define PROB_OCCUPIED eBVM_OCCUPIED



class Box
{
public:
    double x;
    double y;
    double z;
    double sl;
    Box(double x_, double y_, double z_, double side_length_)
        : x(x_), y(y_), z(z_), sl(side_length_)
        {};
    
    Vector3f lower() const
        {
            return Vector3f(x, y, z);
        };
    Vector3f upper() const
        {
            return Vector3f(x+sl, y+sl, z+sl);
        };
};

class BoxWorld
{
public:
    BoxWorld();
    ~BoxWorld();
    
    bool updateActual(const Box &b, bool create_obstacle = true);

    bool isActualCollision();

    bool querySeenState(const Box &b);

    // void querySeenPath();

    void addQueryState(const Box &b);

    void resetQuery();

    size_t countSeenCollisionsInQuery();

    std::vector<size_t> countSeenCollisionsInQueryForEach();

    size_t getNumOccupiedVoxels(const std::string& map_name);

    size_t countIntersect(const std::string& map_1, const std::string& map_2);

    std::vector<size_t> seenSizes();

    void initializeObstacles();

    void doVis();

    Box stateToBox(const ompl::base::State *state) const;


public:
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    int num_observed_sets;
};



class BoxValidator : public ompl::base::StateValidityChecker,
                     public ompl::base::MotionValidator
{
public:
    BoxValidator(const ompl::base::SpaceInformationPtr &si,
                 BoxWorld* box_world);
    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

    BoxWorld* box_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};

class BoxPathValidator : public ompl::geometric::PathValidator
{
public:
    BoxPathValidator(ompl::base::SpaceInformationPtr si,
                     BoxWorld* box_world);
    virtual bool checkPath(const std::vector<ompl::base::State*> path,
                           size_t &collision_index);
    void setProbabilityThreshold(double th);

protected:
    BoxWorld* box_world_ptr;
    double threshold;
};


class BoxMinVoxObjective : public ompl::base::OptimizationObjective
{
public:
    BoxMinVoxObjective(ompl::base::SpaceInformationPtr si,
                       BoxWorld* box_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    BoxWorld* box_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};


class BoxMinColProbObjective : public ompl::base::OptimizationObjective
{
    public:
    BoxMinColProbObjective(ompl::base::SpaceInformationPtr si,
                           BoxWorld* box_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    BoxWorld* box_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};


class BoxMinColProbSweptObjective : public ompl::base::OptimizationObjective
{
    public:
    BoxMinColProbSweptObjective(ompl::base::SpaceInformationPtr si,
                           BoxWorld* box_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    BoxWorld* box_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};



class BoxPlanner
{
public:
    BoxPlanner(BoxWorld* box_world);
    
    bool executePath(ompl::geometric::PathGeometric* path);

    void probeAround(const Box &b, double delta);

    virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                       ompl::base::ScopedState<> goal);

    Maybe::Maybe<ompl::base::PathPtr> planPathDouble(std::vector<double> start,
                                                     std::vector<double> goal);

    virtual void initializePlanner() = 0;

    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);

    virtual void postPlan(ompl::base::PathPtr) {};

public:

    BoxWorld* box_world_ptr;

    std::shared_ptr<ompl::base::Planner> planner;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space;
    ompl::base::SpaceInformationPtr spi_ptr;
    std::shared_ptr<BoxValidator> v_ptr;
    std::shared_ptr<ompl::geometric::PathSimplifier> simp_;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef_;
};

class BoxLBKPIECE : public BoxPlanner
{
public:
    BoxLBKPIECE(BoxWorld* box_world);
    virtual void initializePlanner();
    virtual void postPlan(ompl::base::PathPtr path);
};

template <class T>
class BoxRRTstar : public BoxPlanner
{
public:
    BoxRRTstar(BoxWorld* box_world);
    virtual void initializePlanner();
    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
private:
    std::shared_ptr<ompl::base::OptimizationObjective> objective_ptr;
};


class BoxLazyRRTF : public BoxPlanner
{
public:
    BoxLazyRRTF(BoxWorld* box_world);
    virtual void initializePlanner();
    virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                       ompl::base::ScopedState<> goal);
protected:
    std::shared_ptr<BoxPathValidator> pv_;
};

template <class T>
class BoxTRRT : public BoxPlanner
{
public:
    BoxTRRT(BoxWorld* box_world);
    virtual void initializePlanner();
    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
private:
    std::shared_ptr<ompl::base::OptimizationObjective> objective_ptr;
};





/***********************************************
 **               Box TRRT                    **
 ***********************************************/

template <class T>
BoxTRRT<T>::BoxTRRT(BoxWorld* box_world) :
    BoxPlanner(box_world)
{
    initializePlanner();
    objective_ptr = std::make_shared<T>(spi_ptr, box_world);
    objective_ptr -> setCostThreshold(ompl::base::Cost(0.01));
}

template <class T>
void BoxTRRT<T>::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    
    // planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner = std::make_shared<ompl::geometric::TRRT>(spi_ptr);
    planner->setup();
}

template <class T>
void BoxTRRT<T>::preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ompl::base::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(objective_ptr);
    planner->setProblemDefinition(pdef_);

}



#endif
