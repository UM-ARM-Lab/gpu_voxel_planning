#include "gpu_voxel_planning/scenarios/real_scenario.hpp"
#undef likely  // boost::likely conflict with arc_helpers
#include <arm_pointcloud_utilities/load_save_to_file.h>
#include <hjson/hjson.h>
#include "gpu_voxel_planning/json_helpers.h"

using namespace GVP;

static ObstacleConfiguration getBonkersBelief() {
  ObstacleConfiguration oc;
  Object chair;

  chair.add(AABB(Vector3f(1.0, 1.0, 0.0), Vector3f(1.05, 1.05, 1.2)));
  chair.add(AABB(Vector3f(1.5, 1.0, 0.0), Vector3f(1.55, 1.05, 1.2)));
  chair.add(AABB(Vector3f(1.0, 1.5, 0.0), Vector3f(1.05, 1.55, 0.6)));
  chair.add(AABB(Vector3f(1.5, 1.5, 0.0), Vector3f(1.55, 1.55, 0.6)));
  chair.add(AABB(Vector3f(1.0, 1.0, 0.55), Vector3f(1.55, 1.55, 0.6)));

  chair.add(AABB(Vector3f(1.0, 1.0, 1.1), Vector3f(1.5, 1.05, 1.2)));

  chair.shift(Vector3f(0.0, 2.0, 0));

  oc.add(chair);
  return oc;
}

RealScenario::RealScenario(const std::string &config_file) : s(victor) {
  auto obj = Hjson::UnmarshalFromFile(ros::package::getPath("gpu_voxel_planning") + "/config/" + config_file);
  s.current_config = VictorRightArmConfig(Json::toVector<double>(obj.at("initial_configuration"))).asMap();

  try {
    setKnownGoalConfig(VictorRightArmConfig(Json::toVector<double>(obj.at("goal_configuration"))).asMap());
  } catch (Hjson::index_out_of_bounds &e) {
    std::cout << "No goal configuration set. I hope the belief defines a goal\n";
  }
}

void RealScenario::initFakeVictor(RosInterface& ri) {
  // NOTE:!!! These values are duplicated, and harded coded elsewhere in this code
  // VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0 ,0});
  // ri.setLeftArm(lac);
  // ri.setRightGripper(1.5);
}

void RealScenario::setPrior(ObstacleConfiguration& unknown_obstacles, BeliefParams bp)
// void RealScenario::setPrior(BeliefParams bp)
{
  belief_name = bp.toString();
  switch (bp.belief_type) {
    case BeliefType::CHS:
      std::cout << "Using CHS belief\n";
      s.bel = std::make_unique<ChsBelief>();
      break;
    case BeliefType::Obstacle:
      std::cout << "Using Obstacle belief\n";
      s.bel = std::make_unique<ObstacleBelief>(unknown_obstacles, bp.noise, bp.bias);
      break;
    case BeliefType::Bonkers:
      std::cout << "Using Bonkers belief\n";
      s.bel = std::make_unique<ObstacleBelief>(getBonkersBelief(), bp.noise, bp.bias);
      break;
    case BeliefType::MoEObstacle:
      std::cout << "Using MoE Obstalcebelief\n";
      s.bel = std::make_unique<MoEBelief>(unknown_obstacles, bp.noise, bp.bias);
      break;
    case BeliefType::MoEBonkers:
      std::cout << "Using MoE Obstalcebelief\n";
      s.bel = std::make_unique<MoEBelief>(getBonkersBelief(), bp.noise, bp.bias);
      break;
    // case BeliefType::IID:
    //     std::cout << "Using IID belief\n";
    //     s.bel = std::make_unique<IIDBelief>(unknown_obstacles, bp.noise, bp.bias);
    //     break;
    default:
      std::cout << "Invalid belief type " << bp.belief_type << "\n";
      throw std::invalid_argument("Invalid belief type");
  }
}

void RealScenario::validate() {
  s.robot.set(s.getCurConfig().asMap());
  if (s.robot.occupied_space.overlapsWith(&known_obstacles.occupied)) {
    std::cerr << "Start configuration overlaps with obstacle\n";
    throw(std::invalid_argument("Start configuration is invalid\n"));
  }

  if (known_goal_config.has_value()) {
    auto goals = getPossibleGoals();
    assert(goals.size() == 1);
    s.robot.set(goals.at(0));
    //        s.robot.set(goal_config);
    if (s.robot.occupied_space.overlapsWith(&known_obstacles.occupied)) {
      std::cerr << "Goal configuration overlaps with obstacle\n";
      throw(std::invalid_argument("Goal configuration is invalid\n"));
    }
  }
}

void RealScenario::viz(const GpuVoxelRvizVisualizer& viz) {
  // viz.vizGrid(getTrueObstacles(), "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5));
  viz.vizGrid(s.known_obstacles, "known_obstacles", makeColor(0, 0, 0, 1));

  std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
  viz.vizGrid(s.robot_self_collide_obstacles, "passive_robot", robot_color);
  viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);

  viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);
  s.bel->viz(viz);
}

void RealScenario::addLeftArm() {
  VictorLeftArmAndBase left;
  VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0, 0});
  left.set(lac.asMap());
  s.robot_self_collide_obstacles.add(&left.occupied_space);

  robot::JointValueMap jvm;
  jvm["victor_right_gripper_fingerA_joint_2"] = 1.5;
  jvm["victor_right_gripper_fingerB_joint_2"] = 1.5;
  jvm["victor_right_gripper_fingerC_joint_2"] = 1.5;
  victor.set(jvm);
}

DenseGrid RealScenario::loadPointCloudFromFile() {
  const std::pair<std::string, Eigen::Matrix3Xf> deserialized = arm_pointcloud_utilities::LoadPointsetFromFile(
      ros::package::getPath("arm_pointcloud_utilities") + "/logs/point_cloud_latest.compressed");

  const Eigen::Matrix3Xf& mat = deserialized.second;
  std::cout << "Loading matrix of size " << mat.rows() << ", " << mat.cols() << "\n";
  std::vector<Vector3f> points;
  for (size_t i = 0; i < mat.cols(); i++) {
    if (std::isnan(mat(0, i)) || std::isnan(mat(1, i)) || std::isnan(mat(2, i))) {
      continue;
    }
    // std::cout << mat(0, i) << ", " << mat(1,i) << ", " << mat(2,i) << "\n";
    Vector3f p(1.0 + mat(0, i), 2.0 + mat(1, i), mat(2, i));

    // right arm
    // if(p.x < 1.7 && p.y < 2.0 && p.z > 1.0)
    //     continue;

    // if(p.z > 0.5 && p.z < 1.05 &&
    //    p.y > 1.6)
    // {
    //     continue;
    // }

    if (p.y > 2.3) {
      continue;
    }

    // no point cloud
    // continue;

    points.push_back(p);
  }
  DenseGrid g;
  g.insertPointCloud(points, PROB_OCCUPIED);
  return g;
}

/****************************************
 **         Real Empty
 ***************************************/
RealEmpty::RealEmpty(BeliefParams bp) : RealScenario("real_empty.json"),
                                        name("Fridge") {
  addLeftArm();

  unknown_obstacles.add(getTable());

  setPrior(unknown_obstacles, bp);
  // setPrior(bp);

//  DenseGrid kinect = loadPointCloudFromFile();
  // s.known_obstacles.add(&kinect);

  for (auto& ob : known_obstacles.obstacles) {
    s.known_obstacles.add(&ob.occupied);
  }

}

Object RealEmpty::getTable() {
  // Object table; //table is actually the frige wall now
  // // Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
  // // Vector3f tc(1.7, 1.4, 0.9); //table corner
  // // Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
  // // Vector3f tld(.033, 0.033, tc.z); //table leg dims
  // Vector3f td(2.0 * 0.0254, 12.0 * 0.0254, 24.0 * 0.0254); //table dimensions
  // Vector3f tc(2.5, 2.0, 0.9); //table corner
  // Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
  // Vector3f tld(.033, 0.033, tc.z); //table leg dims

  // table.add(AABB(tc, tc+td));
  // // table.add(AABB(tcf, tcf+tld));
  // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
  // //                Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
  // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
  // //                Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
  // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
  // //                Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
  // return table;

  Object table;
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tcf(1.7, 1.4, 0.0);                              // table corner at floor
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  table.add(AABB(tc, tc + td));
  table.add(AABB(tcf, tcf + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0) + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0) + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0) + tld));
  return table;
}

/****************************************
 **         RealTable
 ****************************************/
RealTable::RealTable(BeliefParams bp) : RealScenario("real_table.json"), name("RealTable") {
  addLeftArm();

  // known_obstacles.add(getTable());

  setPrior(unknown_obstacles, bp);
  // setPrior(bp);

  for (auto& ob : known_obstacles.obstacles) {
    // s.known_obstacles.add(&ob.occupied);
  }
}

Object RealTable::getTable() {
  Object table;
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tcf(1.7, 1.4, 0.0);                              // table corner at floor
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  table.add(AABB(tc, tc + td));
  table.add(AABB(tcf, tcf + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0) + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0) + tld));
  table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0),
                 Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0) + tld));
  return table;
}
