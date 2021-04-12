#ifndef GVP_GOAL_HPP
#define GVP_GOAL_HPP

#include <gpu_voxel_planning_msgs/TSR.h>

#include <jacobian_follower/jacobian_follower.hpp>
#include <utility>

#include "gpu_voxel_planning/robot/robot_model.hpp"

namespace GVP {

class Scenario;

class Goal {
 public:
  //  virtual bool isAchieved(VictorRightArmConfig config) const = 0;
  [[nodiscard]] virtual bool isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const = 0;

  [[nodiscard]] virtual std::vector<robot::JointValueMap> sampleGoalConfigs(const Scenario* scenario) const = 0;
};

class ConfigGoal : public Goal {
 public:
  VictorRightArmConfig goal_config;
  explicit ConfigGoal(VictorRightArmConfig goal_config_) : goal_config(std::move(goal_config_)) {}

  [[nodiscard]] bool isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const override;

  [[nodiscard]] std::vector<robot::JointValueMap> sampleGoalConfigs(const Scenario* scenario) const override;
};

class TSRBound {
 public:
  double min;
  double max;
  TSRBound(double min_, double max_);
  [[nodiscard]] bool contains(double v) const { return min <= v and v <= max; };
};

class TSR {
 public:
  TSRBound x, y, z, rx, ry, rz;
  TSR(TSRBound x_bound, TSRBound y_bound, TSRBound z_bound, TSRBound rx_bound, TSRBound ry_bound, TSRBound rz_bound);

  explicit TSR(const gpu_voxel_planning_msgs::TSR& tsr);

  [[nodiscard]] bool contains(const geometry_msgs::Pose& pose) const;
};

class TSRGoal : public Goal {
 public:
  TSR goal_tsr;

  explicit TSRGoal(const gpu_voxel_planning_msgs::TSR& tsr);

  //  TSRGoal(const gpu_voxel_planning_msgs::TSR& tsr, std::shared_ptr<JacobianFollower> jacobian_follower);
  [[nodiscard]] bool isAchieved(const VictorRightArmConfig& config, const Scenario* scenario) const override;

  [[nodiscard]] std::vector<robot::JointValueMap> sampleGoalConfigs(const Scenario* scenario) const override;

  //  bool tmp(const VictorRightArmConfig& config, const Scenario* scenario) const {
  //    return false;
  //  };
};
}  // namespace GVP

#endif