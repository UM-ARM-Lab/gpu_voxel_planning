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
  // TODO: Check orientation (requires converting pose quaterion to TSR)
  return x.contains(pose.position.x) and y.contains(pose.position.y) and z.contains(pose.position.z);
}

bool ConfigGoal::isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const { return goal_config == config; }

std::vector<robot::JointValueMap> ConfigGoal::sampleGoalConfigs(const Scenario* scenario) const {
  return std::vector<robot::JointValueMap>{goal_config.asMap()};
}

TSRGoal::TSRGoal(const gpu_voxel_planning_msgs::TSR& tsr) : goal_tsr(tsr) {}

bool TSRGoal::isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const {
//  throw std::logic_error("Not implemented");
  auto cur_angles = config.asVector();
  auto cur_pose = scenario->jacobian_follower.computeFK(cur_angles, "right_arm");
  return(goal_tsr.contains(cur_pose));
}

std::vector<robot::JointValueMap> TSRGoal::sampleGoalConfigs(const Scenario* scenario) const {
  std::vector<robot::JointValueMap> goal_configs;
  //
  // TODO: Sample multiple points (perhaps uniformly) from the TSR

  // TODO: Remove hardcoded orientation
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = -0.05594805960241513;
  target_pose.orientation.y = -0.7682472566147173;
  target_pose.orientation.z = -0.6317937464624142;
  target_pose.orientation.w = 0.08661771909760922;

//  auto tsr = dynamic_cast<TSRGoal*>(tsrgoal.get())->goal;


  target_pose.position.x = (goal_tsr.x.min + goal_tsr.x.max) / 2;
  target_pose.position.y = (goal_tsr.y.min + goal_tsr.y.max) / 2;
  target_pose.position.z = (goal_tsr.z.min + goal_tsr.z.max) / 2;

  auto ik_solutions = scenario->jacobian_follower.compute_IK_solutions(target_pose, "right_arm");
  for (const auto& sol : ik_solutions) {
    goal_configs.emplace_back(VictorRightArmConfig(sol).asMap());
  }
  return goal_configs;
}
