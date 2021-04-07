#ifndef WIP_OPTIMIZAION_OBJECTIVE
#define WIP_OPTIMIZAION_OBJECTIVE

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "gpu_voxels_victor.hpp"

class WipOptimizationObjective : public ompl::base::OptimizationObjective {
 public:
  WipOptimizationObjective(ompl::base::SpaceInformationPtr si, std::shared_ptr<GpuVoxelsVictor> victor_model);

  virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;

  virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;

 private:
  std::shared_ptr<GpuVoxelsVictor> victor_model_;
  ompl::base::SpaceInformationPtr si_;
};

#endif
