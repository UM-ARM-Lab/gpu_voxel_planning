#ifndef GPU_PLANNING_ROBOT_MODEL_HPP
#define GPU_PLANNING_ROBOT_MODEL_HPP

#include <ros/package.h>

#include <arc_utilities/timing.hpp>
#include <utility>

#include "gpu_voxel_planning/common_names.hpp"
#include "gpu_voxel_planning/hacky_functions.hpp"
#include "gpu_voxel_planning/maps/prob_map.hpp"

namespace GVP {
class Robot {
 public:
  DenseGrid occupied_space;
  robot::UrdfRobot robot_chain;

  explicit Robot(const std::string& path_to_urdf_file) : robot_chain(VOXEL_SIDE_LENGTH, path_to_urdf_file, false) {
    robot::JointValueMap jvm;
    robot_chain.setConfiguration(jvm);
    occupied_space.insertMetaPointCloud(*robot_chain.getTransformedClouds(), PROB_OCCUPIED);
  }

  void set(const robot::JointValueMap& map) {
    PROFILE_START("Set robot config");
    robot_chain.setConfiguration(map);
    occupied_space.clearMap();
    occupied_space.insertMetaPointCloud(*robot_chain.getTransformedClouds(), PROB_OCCUPIED);
    PROFILE_RECORD("Set robot config");
  }

  virtual bool isValid() { return true; }

  virtual std::vector<std::string> getLinkNames() const = 0;

  virtual std::vector<DenseGrid> getLinkOccupancies() = 0;

  DenseGrid getLinkOccupancy(const std::string& link_name) {
    DenseGrid g;
    const MetaPointCloud* clouds = robot_chain.getTransformedClouds();
    robot_chain.syncToHost();

    int16_t cloud_num = clouds->getCloudNumber(link_name);
    uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
    const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
    const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
    g.insertPointCloud(cloud, PROB_OCCUPIED);
    return g;
  }
};

class VictorArmConfig {
 public:
  const std::vector<std::string>* joint_names;
  std::vector<double> joint_values;

  explicit VictorArmConfig(const std::vector<std::string>* joint_names) : joint_names(joint_names){};

  VictorArmConfig(const std::vector<std::string>* joint_names, std::vector<double> joint_values_)
      : joint_names(joint_names), joint_values(std::move(joint_values_)){
      if(joint_values.size() != joint_names->size()){
          throw std::range_error("Invalid joint size when creating VictorArmConfig: " + std::to_string(joint_values.size()));
      }
  };

  VictorArmConfig(const std::vector<std::string>* joint_names, const double* values) : joint_names(joint_names) {
    for (size_t i = 0; i < joint_names->size(); i++) {
      joint_values.push_back(values[i]);
    }
  }

  VictorArmConfig(const std::vector<std::string>* joint_names, const robot::JointValueMap& jvm)
      : joint_names(joint_names) {
    for (const auto& joint_name : *joint_names) {
      joint_values.push_back(jvm.at(joint_name));
    }
  }

  [[nodiscard]] robot::JointValueMap asMap() const {
    robot::JointValueMap jvm;
    for (size_t i = 0; i < joint_names->size(); i++) {
      jvm[joint_names->at(i)] = joint_values[i];
    }
    return jvm;
  }

  [[nodiscard]] std::vector<double> asVector() const { return joint_values; }

  bool operator==(const VictorArmConfig& other) const {
    if (joint_values.size() != other.joint_values.size()) {
      return false;
    }

    for (size_t i = 0; i < joint_values.size(); i++) {
      if (fabs(joint_values[i] - other.joint_values[i]) > 0.0001) {
        return false;
      }
      // if(joint_names[i] != other.joint_names[i])
      // {
      //     return false;
      // }
    }
    return true;
  }

  bool operator!=(const VictorArmConfig& other) const { return !(operator==(other)); }
};

class VictorRightArmConfig : public VictorArmConfig {
 public:
  VictorRightArmConfig() : VictorArmConfig(&right_arm_joint_names) {}

  explicit VictorRightArmConfig(std::vector<double> joint_values)
      : VictorArmConfig(&right_arm_joint_names, std::move(joint_values)) {}

  explicit VictorRightArmConfig(const double* values) : VictorArmConfig(&right_arm_joint_names, values) {}

  explicit VictorRightArmConfig(const robot::JointValueMap& jvm) : VictorArmConfig(&right_arm_joint_names, jvm) {}
};

class VictorLeftArmConfig : public VictorArmConfig {
 public:
  VictorLeftArmConfig() : VictorArmConfig(&left_arm_joint_names) {}

  explicit VictorLeftArmConfig(std::vector<double> joint_values)
      : VictorArmConfig(&left_arm_joint_names, std::move(joint_values)) {}

  explicit VictorLeftArmConfig(const double* values) : VictorArmConfig(&left_arm_joint_names, values) {}

  explicit VictorLeftArmConfig(const robot::JointValueMap& jvm) : VictorArmConfig(&left_arm_joint_names, jvm) {}
};

class VictorRightArm : public Robot {
 public:
  const std::vector<std::string> collision_link_names{
      "victor_right_arm_link_2", "victor_right_arm_link_3", "victor_right_arm_link_4",
      "victor_right_arm_link_5", "victor_right_arm_link_6", "victor_right_arm_link_7",
  };

  const std::vector<std::string> gripper_link_names{
      "victor_right_gripper_palm",         "victor_right_gripper_mounting_bracket", "victor_right_gripper_fingerA_base",
      "victor_right_gripper_fingerA_dist", "victor_right_gripper_fingerA_med",      "victor_right_gripper_fingerA_prox",
      "victor_right_gripper_fingerB_base", "victor_right_gripper_fingerB_dist",     "victor_right_gripper_fingerB_med",
      "victor_right_gripper_fingerB_prox", "victor_right_gripper_fingerC_base",     "victor_right_gripper_fingerC_dist",
      "victor_right_gripper_fingerC_med",  "victor_right_gripper_fingerC_prox"};

  VictorRightArm() : Robot(ros::package::getPath("gpu_voxel_planning") + "/urdf/victor_right_arm_only.urdf") {}

  std::vector<std::string> getLinkNames() const override { return collision_link_names; }

  std::vector<DenseGrid> getLinkOccupancies() override {
    std::vector<DenseGrid> link_occupancies;
    for (const std::string& link : collision_link_names) {
      link_occupancies.push_back(getLinkOccupancy(link));
    }
    for (const std::string& gripper_link : gripper_link_names) {
      DenseGrid tmp = getLinkOccupancy(gripper_link);
      link_occupancies.back().add(&tmp);
    }
    return link_occupancies;
  }
};

class VictorLeftArmAndBase : public Robot {
 public:
  VictorLeftArmAndBase() : Robot(ros::package::getPath("gpu_voxel_planning") + "/urdf/victor_left_arm_and_body.urdf") {}

  std::vector<std::string> getLinkNames() const override { throw std::logic_error("Not implemented for left arm"); }

  std::vector<DenseGrid> getLinkOccupancies() override { throw std::logic_error("Not implemented for left arm"); }
};
}  // namespace GVP

#endif
