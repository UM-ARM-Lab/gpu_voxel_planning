#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP

#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "gpu_voxel_planning/state.hpp"

#include <jacobian_follower/jacobian_follower.hpp>

namespace GVP {
class Scenario {
 public:
  explicit Scenario(): jacobian_follower("victor", 0.01) {}

  VictorRightArm victor;
  //        robot::JointValueMap goal_config;
  std::optional<robot::JointValueMap> known_goal_config;
  JacobianFollower jacobian_follower;

  void setKnownGoalConfig(robot::JointValueMap goal_config) { known_goal_config = goal_config; }

  virtual State &getState() = 0;

  virtual const State &getState() const = 0;

  virtual std::string getName() const = 0;

  virtual std::vector<robot::JointValueMap> getPossibleGoals() const {
    if (known_goal_config.has_value()) {
      return std::vector<robot::JointValueMap>{known_goal_config.value()};
    }
    throw std::runtime_error("Scenario does not have a known_goal_config");
  }

  virtual bool completed() const {
    for (const auto &goal : getPossibleGoals()) {
      if (VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal)) {
        return true;
      }
    }
    std::cout << PrettyPrint::PrettyPrint(VictorRightArmConfig(getState().current_config).asVector(), true)
              << " is not a goal\n";
    return false;
    //            return VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal_config);
  }

  virtual void viz(const GpuVoxelRvizVisualizer &viz) {}
};
}  // namespace GVP

#endif
