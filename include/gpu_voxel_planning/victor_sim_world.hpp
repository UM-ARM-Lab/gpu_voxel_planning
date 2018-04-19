#ifndef VICTOR_WORLD_HPP
#define VICTOR_WORLD_HPP


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



const std::string VICTOR_ACTUAL_MAP = "victor_actual";
const std::string VICTOR_QUERY_MAP = "victor_query";
const std::string VICTOR_SWEPT_VOLUME_MAP = "victor_swept_volume";
const std::string OBSTACLES_ACTUAL_MAP = "actual_obstacles";
const std::string FULL_MAP = "full_map";
// const std::string OBSTACLES_SEEN_MAP = "seen_obstacles";

std::vector<std::string> SEEN_OBSTACLE_SETS;


#define PROB_OCCUPIED eBVM_OCCUPIED

typedef robot::JointValueMap VictorConfig;

class Box
{
public:
    double x;
    double y;
    double z;
    double x_side;
    double y_side;
    double z_side;
    Box(double x_, double y_, double z_, double x_side_, double y_side_, double z_side_)
        : x(x_), y(y_), z(z_), x_side(x_side_), y_side(y_side_), z_side(z_side_)
        {};
    
    Vector3f lower() const
        {
            return Vector3f(x, y, z);
        };
    Vector3f upper() const
        {
            return Vector3f(x+x_side, y+y_side, z+z_side);
        };
};


class VictorWorld
{
public:
    VictorWorld();
    ~VictorWorld();

    void insertVictorIntoMap(const VictorConfig &c, const std::string &map_name);
    
    bool updateActual(const VictorConfig &b, bool create_obstacle = true);

    bool isActualCollision();

    bool querySeenState(const VictorConfig &b);

    // void querySeenPath();

    void addQueryState(const VictorConfig &b);

    void resetQuery();

    size_t countSeenCollisionsInQuery();

    std::vector<size_t> countSeenCollisionsInQueryForEach();

    size_t getNumOccupiedVoxels(const std::string& map_name);

    size_t countIntersect(const std::string& map_1, const std::string& map_2);

    std::vector<size_t> seenSizes();

    void initializeObstacles();

    void doVis();

    VictorConfig stateToVictorConfig(const ompl::base::State *state) const;


public:
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    int num_observed_sets;
};



class VictorValidator : public ompl::base::StateValidityChecker,
                        public ompl::base::MotionValidator
{
public:
    VictorValidator(const ompl::base::SpaceInformationPtr &si,
                 VictorWorld* victor_world);
    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

    VictorWorld* victor_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};

class VictorPathValidator : public ompl::geometric::PathValidator
{
public:
    VictorPathValidator(ompl::base::SpaceInformationPtr si,
                     VictorWorld* victor_world);
    virtual bool checkPath(const std::vector<ompl::base::State*> path,
                           size_t &collision_index);
    void setProbabilityThreshold(double th);

protected:
    VictorWorld* victor_world_ptr;
    double threshold;
};


class VictorMinVoxObjective : public ompl::base::OptimizationObjective
{
public:
    VictorMinVoxObjective(ompl::base::SpaceInformationPtr si,
                       VictorWorld* victor_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    VictorWorld* victor_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};


class VictorMinColProbObjective : public ompl::base::OptimizationObjective
{
    public:
    VictorMinColProbObjective(ompl::base::SpaceInformationPtr si,
                           VictorWorld* victor_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    VictorWorld* victor_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};


class VictorMinColProbSweptObjective : public ompl::base::OptimizationObjective
{
    public:
    VictorMinColProbSweptObjective(ompl::base::SpaceInformationPtr si,
                           VictorWorld* victor_world);
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const; 
    
private:
    VictorWorld* victor_world_ptr;
    ompl::base::SpaceInformationPtr spi_ptr;
};



class VictorPlanner
{
public:
    VictorPlanner(VictorWorld* victor_world);
    
    bool executePath(ompl::geometric::PathGeometric* path);

    void probeAround(const VictorConfig &b, double delta);

    virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                       ompl::base::ScopedState<> goal);

    Maybe::Maybe<ompl::base::PathPtr> planPathDouble(std::vector<double> start,
                                                     std::vector<double> goal);

    virtual void initializePlanner() = 0;

    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);

    virtual void postPlan(ompl::base::PathPtr) {};

public:

    VictorWorld* victor_world_ptr;

    std::shared_ptr<ompl::base::Planner> planner;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space;
    ompl::base::SpaceInformationPtr spi_ptr;
    std::shared_ptr<VictorValidator> v_ptr;
    std::shared_ptr<ompl::geometric::PathSimplifier> simp_;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef_;
};

class VictorLBKPIECE : public VictorPlanner
{
public:
    VictorLBKPIECE(VictorWorld* victor_world);
    virtual void initializePlanner();
    virtual void postPlan(ompl::base::PathPtr path);
};

template <class T>
class VictorRRTstar : public VictorPlanner
{
public:
    VictorRRTstar(VictorWorld* victor_world);
    virtual void initializePlanner();
    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
private:
    std::shared_ptr<ompl::base::OptimizationObjective> objective_ptr;
};


class VictorLazyRRTF : public VictorPlanner
{
public:
    VictorLazyRRTF(VictorWorld* victor_world);
    virtual void initializePlanner();
    virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                       ompl::base::ScopedState<> goal);
    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
protected:
    std::shared_ptr<VictorPathValidator> pv_;
    double threshold;
};

template <class T>
class VictorTRRT : public VictorPlanner
{
public:
    VictorTRRT(VictorWorld* victor_world);
    virtual void initializePlanner();
    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
private:
    std::shared_ptr<ompl::base::OptimizationObjective> objective_ptr;
};





/***********************************************
 **               Victor TRRT                    **
 ***********************************************/

template <class T>
VictorTRRT<T>::VictorTRRT(VictorWorld* victor_world) :
    VictorPlanner(victor_world)
{
    initializePlanner();
    objective_ptr = std::make_shared<T>(spi_ptr, victor_world);
    objective_ptr -> setCostThreshold(ompl::base::Cost(0.01));
}

template <class T>
void VictorTRRT<T>::initializePlanner()
{
    // spi_ptr->setStateValidityChecker(v_ptr);
    // spi_ptr->setMotionValidator(v_ptr);
    
    // planner = std::make_shared<og::RRTstar>(spi_ptr);
    planner = std::make_shared<ompl::geometric::TRRT>(spi_ptr);
    planner->setup();
}

template <class T>
void VictorTRRT<T>::preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
{
    planner->clear();
    pdef_ = std::make_shared<ompl::base::ProblemDefinition>(spi_ptr);
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(objective_ptr);
    planner->setProblemDefinition(pdef_);

}



#endif
