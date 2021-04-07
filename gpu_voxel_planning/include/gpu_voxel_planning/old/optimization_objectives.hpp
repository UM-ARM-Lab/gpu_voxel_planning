#ifndef VICTOR_VALIDATOR_INCLUDED
#define VICTOR_VALIDATOR_INCLUDED

#include <ompl/base/SpaceInformation.h>

#include "gpu_voxels_victor.hpp"
// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/config.h>

#include <iostream>
#include <mutex>
#include <tuple>

class MinVoxelsObjective : public ompl::base::OptimizationObjective {
 public:
  MinVoxelsObjective(const ompl::base::SpaceInformationPtr &si);
  ~MinVoxelsObjective();

  virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;
  virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;
};

#endif
