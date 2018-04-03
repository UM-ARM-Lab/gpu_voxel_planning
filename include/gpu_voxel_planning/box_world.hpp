#ifndef BOX_WORLD_HPP
#define BOX_WORLD_HPP


#include <gpu_voxels/GpuVoxels.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <string>



const std::string BOX_ACTUAL_MAP = "box_actual";
const std::string BOX_QUERY_MAP = "box_query";
const std::string OBSTACLES_ACTUAL_MAP = "actual_obstacles";
const std::string OBSTACLES_SEEN_MAP = "seen_obstacles";

#define PROB_OCCUPIED eBVM_OCCUPIED


struct Position
{
    double x;
    double y;
    double z;
    Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_)
        {
        }
};

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
    
    void updateActual(const Box &b);

    bool querySeenState(const Box &b);

    // void querySeenPath();

    void addQueryState(const Box &b);

    void resetQuery();

    size_t countSeenCollisionsInQuery();

    void initializeObstacles();

    void doVis();

    Box stateToBox(const ompl::base::State *state) const;


public:
    gpu_voxels::GpuVoxelsSharedPtr gvl;
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
    ompl::base::SpaceInformationPtr si_ptr;
        
};



class BoxPlanner
{
public:
    BoxPlanner(BoxWorld* box_world);
    
    bool executePath(ompl::geometric::PathGeometric* path);

    ompl::base::PathPtr planPath(ompl::base::ScopedState<> start,
                                 ompl::base::ScopedState<> goal);

    ompl::base::PathPtr planPath(std::vector<double> start,
                                 std::vector<double> goal);

    virtual void preparePlanner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);

public:

    BoxWorld* box_world_ptr;

    ompl::base::Planner* planner;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space;
    ompl::base::SpaceInformationPtr si_ptr;
    std::shared_ptr<BoxValidator> v_ptr;
    std::shared_ptr<ompl::geometric::PathSimplifier> simp_;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef_;
};

#endif
