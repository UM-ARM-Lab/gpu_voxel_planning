#include <ros/ros.h>

#include "collision_detection.hpp"
#include "std_msgs/String.h"

ros::Publisher speaker;

void checkCollisionCallback(victor_hardware_interface::MotionStatus::ConstPtr motion_msg) {
  CollisionInformation c = checkCollision(motion_msg);
  if (c.collision) {
    for (size_t i = 0; i < c.joint_pos.size(); i++) {
      std::cout << c.torques[i] << ", ";
    }
    std::cout << "\n";

    std_msgs::String msg;
    msg.data = c.links_in_contact[0].back();
    speaker.publish(msg);
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Check_collision_links");
  ros::NodeHandle nh;

  speaker = nh.advertise<std_msgs::String>("/polly", 1, true);

  // ros::Duration(1.0).sleep();
  ros::Subscriber collision_sub = nh.subscribe("right_arm/motion_status", 1, checkCollisionCallback);
  std_msgs::String msg;
  msg.data = "Looking for collisions";
  speaker.publish(msg);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
