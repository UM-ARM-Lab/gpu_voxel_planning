#include <gpu_voxel_planning_msgs/TSR.h>

#include <jacobian_follower/jacobian_follower.hpp>

#include "gpu_voxel_planning/robot/robot_model.hpp"

namespace GVP {

class Goal {
 public:
  //  virtual bool isAchieved(VictorRightArmConfig config) const = 0;
};

class ConfigGoal : Goal {
 public:
  VictorRightArmConfig goal_config;
  explicit ConfigGoal(VictorRightArmConfig goal_config_) : goal_config(goal_config_) {}

  [[nodiscard]] bool isAchieved(VictorRightArmConfig config) const;
};

class TSRBound {
 public:
  double min;
  double max;
  TSRBound(double min_, double max_);
};

class TSR {
 public:
  TSRBound x, y, z, rx, ry, rz;
  TSR(TSRBound x_bound, TSRBound y_bound, TSRBound z_bound, TSRBound rx_bound, TSRBound ry_bound, TSRBound rz_bound);

  explicit TSR(const gpu_voxel_planning_msgs::TSR& tsr);
};

class TSRGoal : Goal {
 public:
  TSR goal;
  [[nodiscard]] bool isAchieved(VictorRightArmConfig config, JacobianFollower& jf);

  explicit TSRGoal(const gpu_voxel_planning_msgs::TSR& tsr);
};
}  // namespace GVP