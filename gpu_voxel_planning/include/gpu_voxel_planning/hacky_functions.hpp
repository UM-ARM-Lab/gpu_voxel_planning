#ifndef HACKY_HELPERS
#define HACKY_HELPERS

#include <string>
#include <vector>

inline std::vector<double> right_joint_lower_deg = {-170, -120, -170, -120, -170, -120, -175};
inline std::vector<double> right_joint_upper_deg = {170, 120, 170, 120, 170, 120, 175};
inline double torad = 3.1415 / 180;

inline std::vector<std::string> right_arm_joint_names{
    "victor_right_arm_joint_1", "victor_right_arm_joint_2", "victor_right_arm_joint_3", "victor_right_arm_joint_4",
    "victor_right_arm_joint_5", "victor_right_arm_joint_6", "victor_right_arm_joint_7"};

inline std::vector<std::string> right_arm_collision_link_names{"victor_right_arm_link_3",
                                                                     "victor_right_arm_link_4",
                                                                     "victor_right_arm_link_5",
                                                                     "victor_right_arm_link_6",
                                                                     "victor_right_arm_link_7",
                                                                     "victor_right_gripper_palm",
                                                                     "victor_right_gripper_mounting_bracket",
                                                                     "victor_right_gripper_fingerA_base",
                                                                     "victor_right_gripper_fingerA_dist",
                                                                     "victor_right_gripper_fingerA_med",
                                                                     "victor_right_gripper_fingerA_prox",
                                                                     "victor_right_gripper_fingerB_base",
                                                                     "victor_right_gripper_fingerB_dist",
                                                                     "victor_right_gripper_fingerB_med",
                                                                     "victor_right_gripper_fingerB_prox",
                                                                     "victor_right_gripper_fingerC_base",
                                                                     "victor_right_gripper_fingerC_dist",
                                                                     "victor_right_gripper_fingerC_med",
                                                                     "victor_right_gripper_fingerC_prox"};

inline std::vector<std::string> left_arm_joint_names{
    "victor_left_arm_joint_1", "victor_left_arm_joint_2", "victor_left_arm_joint_3", "victor_left_arm_joint_4",
    "victor_left_arm_joint_5", "victor_left_arm_joint_6", "victor_left_arm_joint_7"};

#endif
