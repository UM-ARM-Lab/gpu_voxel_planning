#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include <victor_hardware_interface/MotionStatus.h>

#include <string>

std::vector<double> torque_collision_limits = {5, 5, 5, 4, 2, 1, 1};

struct CollisionInformation {
  bool collision;
  std::vector<double> torques;
  std::vector<int> dirs;
  std::vector<double> joint_pos;
  std::vector<std::string> links_in_contact;
};

std::vector<double> jvqToVector(victor_hardware_interface::JointValueQuantity jvq) {
  std::vector<double> v{jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4, jvq.joint_5, jvq.joint_6, jvq.joint_7};
  return v;
};

CollisionInformation checkCollision(victor_hardware_interface::MotionStatus::ConstPtr motion_msg) {
  CollisionInformation c;

  c.collision = false;
  std::vector<double> joint_torques = jvqToVector(motion_msg->estimated_external_torque);
  // for(auto ext_torque: joint_torques)
  for (size_t i = 0; i < joint_torques.size(); i++) {
    if (fabs(joint_torques[i]) > torque_collision_limits[i]) {
      c.collision = true;
      break;
    }
  }

  if (c.collision) {
    std::cout << "Collision!\n";
    c.torques = jvqToVector(motion_msg->estimated_external_torque);
    c.joint_pos = jvqToVector(motion_msg->measured_joint_position);
    c.dirs.resize(c.joint_pos.size());
    for (size_t i = 0; i < c.torques.size(); i++) {
      double tqr = c.torques[i];
      c.dirs[i] = (0 < -tqr) - (-tqr < 0);  // sgn
    }

    bool distal_link_in_col = false;
    for (int i = (int)joint_torques.size() - 1; i >= 0; i--) {
      if (!distal_link_in_col) {
        std::string link_name = "victor_right_arm_link_" + std::to_string(i + 1);
        c.links_in_contact.push_back(link_name);
        if (fabs(joint_torques[i]) > torque_collision_limits[i]) {
          distal_link_in_col = true;
        }
      }
    }

    std::reverse(std::begin(c.links_in_contact), std::end(c.links_in_contact));
  }

  return c;
};
#endif
