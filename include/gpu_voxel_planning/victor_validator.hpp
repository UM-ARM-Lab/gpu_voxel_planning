#ifndef VICTOR_VALIDATOR_INCLUDED
#define VICTOR_VALIDATOR_INCLUDED

#include <gpu_voxels/GpuVoxels.h>

#include "collision_detection.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <tuple>

#include <mutex>


class VictorValidator : public ompl::base::StateValidityChecker, public ompl::base::MotionValidator, public std::enable_shared_from_this<VictorValidator>
{
public:
    VictorValidator(const ompl::base::SpaceInformationPtr &si);
    ~VictorValidator();

    bool isCurrentlyValid() const;
    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;


    std::shared_ptr<VictorValidator> getptr() {
        return shared_from_this();
    }

    void insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const;
    void visualizeSolution(ompl::base::PathPtr path);
    void doVis();
    void testObstacle();
    void setVictorPosition(robot::JointValueMap joint_positions);
    void addCollisionPoints(CollisionInformation collision_info);

    int determineVictorDist();

public:
    ompl::base::SpaceInformationPtr si_;
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    
private:
    ompl::base::StateSpace *stateSpace_;

    mutable std::mutex g_i_mutex;
    mutable std::mutex g_j_mutex;
};

namespace vvhelpers{
    template<typename T>
    robot::JointValueMap toRightJointValueMap(const T values);

    bool isInJointLimits(const double *values);

}


#endif
