#ifndef VICTOR_VALIDATOR_INCLUDED
#define VICTOR_VALIDATOR_INCLUDED


#include "path_validator.h"
#include "gpu_voxels_victor.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <tuple>

#include <mutex>


class VictorValidator : public ompl::base::StateValidityChecker,
                        public ompl::base::MotionValidator,
                        public std::enable_shared_from_this<VictorValidator>
{
public:
    VictorValidator(const ompl::base::SpaceInformationPtr &si,
                    GpuVoxelsVictor* victor_model);
    ~VictorValidator();

    // bool isCurrentlyValid() const;
    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;


    std::shared_ptr<VictorValidator> getptr() {
        return shared_from_this();
    }


public:
    const ompl::base::SpaceInformationPtr si_;
    
protected:
    ompl::base::StateSpace *stateSpace_;
    GpuVoxelsVictor* victor_model_;
};


class VictorConservativeValidator : public VictorValidator
{
public:
    VictorConservativeValidator(const ompl::base::SpaceInformationPtr &si,
                                GpuVoxelsVictor* victor_model);

    virtual bool isValid(const ompl::base::State *state) const;
};

class VictorPathThresholdValidator : public VictorValidator
{
public:
    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;
};





class VictorPathValidator : public ompl::geometric::PathValidator
{
public:
    VictorPathValidator(const ompl::base::SpaceInformationPtr &si,
                        GpuVoxelsVictor* victor_model);
    virtual bool checkPath(const std::vector<ompl::base::State*> path,
                           size_t &collision_index);
    void setProbabilityThreshold(double th);

protected:
    GpuVoxelsVictor* victor_model_;
    double threshold;
};



#endif
