//
// Created by bsaund on 4/8/21.
//
#include "gpu_voxel_planning/scenarios/goal.hpp"
#include "gpu_voxel_planning/scenarios/scenarios.hpp"

using namespace GVP;

TSRBound::TSRBound(double min_, double max_) : min(min_), max(max_) {
  if (max < min) {
    throw std::invalid_argument("max cannot be less than min");
  }
}

TSR::TSR(TSRBound x_bound, TSRBound y_bound, TSRBound z_bound, TSRBound rx_bound, TSRBound ry_bound, TSRBound rz_bound)
    : x(x_bound), y(y_bound), z(z_bound), rx(rx_bound), ry(ry_bound), rz(rz_bound) {}

TSR::TSR(const gpu_voxel_planning_msgs::TSR& tsr)
    : x(TSRBound(tsr.x_lower, tsr.x_upper)),
      y(TSRBound(tsr.y_lower, tsr.y_upper)),
      z(TSRBound(tsr.z_lower, tsr.z_upper)),
      rx(TSRBound(tsr.rx_lower, tsr.rx_upper)),
      ry(TSRBound(tsr.ry_lower, tsr.ry_upper)),
      rz(TSRBound(tsr.rz_lower, tsr.rz_upper)) {}

bool TSR::contains(const geometry_msgs::Pose& pose) const {
  // TODO: Check orientation (requires converting pose quaterion to TSR
  return x.contains(pose.position.x) and y.contains(pose.position.y) and z.contains(pose.position.z);
}

bool ConfigGoal::isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const { return goal_config == config; }

TSRGoal::TSRGoal(const gpu_voxel_planning_msgs::TSR& tsr) : goal(tsr) {}

bool TSRGoal::isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const {
//  throw std::logic_error("Not implemented");
  auto cur_angles = config.asVector();
  auto cur_pose = scenario->jacobian_follower.computeFK(cur_angles, "right_arm");
  return(goal.contains(cur_pose));
}
