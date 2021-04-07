#ifndef GVP_OMPL_UTILS
#define GVP_OMPL_UTILS

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <vector>

#include "path_utils.hpp"

namespace ompl_utils

{
ompl::base::ScopedState<> toScopedState(std::vector<double> ds,
                                        const std::shared_ptr<ompl::base::RealVectorStateSpace> space) {
  ompl::base::ScopedState<> s(space);
  for (size_t i = 0; i < ds.size(); i++) {
    s[i] = ds[i];
  }
  return s;
};

PathUtils::Path omplPathToDoublePath(ompl::geometric::PathGeometric *ompl_path,
                                     std::shared_ptr<ompl::base::SpaceInformation> si_) {
  PathUtils::Path d_path;

  ompl::base::StateSpace *stateSpace = si_->getStateSpace().get();
  ompl::base::State *state = si_->allocState();
  for (size_t step = 0; step < ompl_path->getStateCount() - 1; step++) {
    ompl::base::State *s1 = ompl_path->getState(step);
    ompl::base::State *s2 = ompl_path->getState(step + 1);
    int nd = stateSpace->validSegmentCount(s1, s2);

    for (int j = 0; j < nd; j++) {
      stateSpace->interpolate(s1, s2, (double)j / (double)nd, state);
      const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<double> d_state;
      for (int i = 0; i < 7; i++) {
        d_state.push_back(values[i]);
      }

      assert(d_state.size() == 7);
      d_path.push_back(d_state);
    }
  }
  ompl::base::State *last = ompl_path->getState(ompl_path->getStateCount() - 1);
  const double *values = last->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  std::vector<double> d_state;
  for (int i = 0; i < 7; i++) {
    d_state.push_back(values[i]);
  }
  d_path.push_back(d_state);

  si_->freeState(state);

  return d_path;
}

ompl::base::ScopedState<> samplePointInRandomDirection(ompl::base::ScopedState<> start,
                                                       std::shared_ptr<ompl::base::SpaceInformation> si_,
                                                       double max_motion) {
  // double max_motion = space->getLongestValidSegmentLength() * 2;

  std::shared_ptr<ompl::base::StateSpace> space = si_->getStateSpace();
  ompl::base::ScopedState<> new_state(space);
  ompl::base::StateSamplerPtr sampler_ = si_->allocStateSampler();
  sampler_->sampleUniform(new_state.get());
  const double *startv = start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double *new_vals = new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for (int i = 0; i < 7; i++) {
    new_vals[i] += startv[i];
  }

  double motion_dist = si_->distance(start.get(), new_state.get());

  if (motion_dist > max_motion) {
    space->interpolate(start.get(), new_state.get(), max_motion / motion_dist, new_state.get());
  }

  return new_state;
}

}  // namespace ompl_utils
#endif
