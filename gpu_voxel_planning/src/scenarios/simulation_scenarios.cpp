#include "gpu_voxel_planning/scenarios/simulation_scenarios.hpp"

#include <gpu_voxel_planning_msgs/RequestShape.h>
#include <hjson/hjson.h>
#include <ros/package.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "gpu_voxel_planning/json_helpers.h"
#include "gpu_voxel_planning/pointcloud_utils.h"

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

SimulationScenario::SimulationScenario(const std::string &config_file) : s(victor) {
  //  Hjson::Value obj;
  //  std::ifstream ifs(ros::package::getPath("gpu_voxel_planning") + "/config/" + config_file);
  //  if(not ifs.is_open()){
  //    throw std::invalid_argument(std::string("Config file not found: ") + config_file);
  //  }
  //  ifs >> obj;
  auto obj = Hjson::UnmarshalFromFile(ros::package::getPath("gpu_voxel_planning") + "/config/" + config_file);

  s.current_config = VictorRightArmConfig(Json::toVector<double>(obj.at("initial_configuration"))).asMap();
  //    goal_config = VictorRightArmConfig(Json::toVector<double>(obj.at("goal_configuration"))).asMap();
  try {
    setKnownGoalConfig(VictorRightArmConfig(Json::toVector<double>(obj.at("goal_configuration"))).asMap());
  } catch (Hjson::index_out_of_bounds &e) {
    std::cout << "No goal configuration set. I hope the belief defines a goal\n";
  }
  addLeftArm();
}

void SimulationScenario::initFakeVictor(RosInterface &ri) {
  // NOTE:!!! These values are duplicated, and hard coded elsewhere in this code
  VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0, 0});
  ri.setLeftArm(lac);
  ri.setRightGripper(1.5);
}

void SimulationScenario::setPrior(ObstacleConfiguration &unknown_obstacles_prior, const BeliefParams &bp) {
  belief_name = bp.toString();
  switch (bp.belief_type) {
    case BeliefType::CHS:
      std::cout << "Using CHS belief\n";
      s.bel = std::make_unique<ChsBelief>();
      break;
    case BeliefType::Obstacle:
      std::cout << "Using Obstacle belief\n";
      s.bel = std::make_unique<ObstacleBelief>(unknown_obstacles_prior, bp.noise, bp.bias);
      break;
    case BeliefType::Bonkers:
      std::cout << "Using Bonkers belief\n";
      s.bel = std::make_unique<ObstacleBelief>(getBonkersBelief(), bp.noise, bp.bias);
      break;
    case BeliefType::MoEObstacle:
      std::cout << "Using MoE Obstacle belief\n";
      s.bel = std::make_unique<MoEBelief>(unknown_obstacles_prior, bp.noise, bp.bias);
      break;
    case BeliefType::MoEBonkers:
      std::cout << "Using MoE Obstacle belief\n";
      s.bel = std::make_unique<MoEBelief>(getBonkersBelief(), bp.noise, bp.bias);
      break;
    case BeliefType::IID:
      std::cout << "Using IID belief\n";
      s.bel = std::make_unique<IIDBelief>(unknown_obstacles_prior, bp.noise, bp.bias);
      break;
    case BeliefType::Deterministic:
      std::cout << "Using Deterministic\n";
      s.bel = std::make_unique<EmptyBelief>();
      break;
    case BeliefType::ShapeCompletion:
      std::cout << "Using shape completion belief\n";
      s.bel = std::make_unique<ShapeCompletionBelief>();
      break;
    default:
      std::cout << "Invalid belief type " << bp.belief_type << "\n";
      throw std::invalid_argument("Invalid belief type");
  }
}

void SimulationScenario::validate() {
  s.robot.set(s.getCurConfig().asMap());
  if (s.robot.occupied_space.overlapsWith(&true_obstacles.occupied)) {
    std::cerr << "Start configuration overlaps with obstacle\n";
    throw(std::invalid_argument("Start configuration is invalid\n"));
  }

  if (known_goal_config.has_value()) {
    auto goals = getPossibleGoals();
    assert(goals.size() == 1);
    s.robot.set(goals.at(0));
    if (s.robot.occupied_space.overlapsWith(&true_obstacles.occupied)) {
      std::cerr << "Goal configuration overlaps with obstacle\n";
      throw(std::invalid_argument("Goal configuration is invalid\n"));
    }
  }
  s.robot.set(s.getCurConfig().asMap());
}

void SimulationScenario::viz(const GpuVoxelRvizVisualizer &viz) {
  viz.vizGrid(getTrueObstacles(), "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5));
  viz.vizGrid(s.known_obstacles, "known_obstacles", makeColor(0, 0, 0, 1));

  std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
  viz.vizGrid(s.robot_self_collide_obstacles, "passive_robot", robot_color);
  viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);
  s.bel->viz(viz);
}

void SimulationScenario::addLeftArm() {
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

void SimulationScenario::combineObstacles() {
  for (auto &ob : known_obstacles.obstacles) {
    s.known_obstacles.add(&ob.occupied);
  }

  for (const auto &ob : known_obstacles.obstacles) {
    true_obstacles.add(ob);
  }
  for (const auto &ob : unknown_obstacles.obstacles) {
    true_obstacles.add(ob);
  }
}

/****************************************
 **         Empty
 ****************************************/
Empty::Empty(const BeliefParams &bp) : SimulationScenario("empty_scenario.json"), name("empty") {
  setPrior(unknown_obstacles, bp);

  //    s.current_config = VictorRightArmConfig(std::vector<double>{-1.0, 1.5, -1.5, 0.4, -1.5, 0.0, 1.5}).asMap();
  //    goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();
}

/****************************************
 **         Table With Box
 ****************************************/
TableWithBox::TableWithBox(const BeliefParams &bp, bool table_known, bool visible_cave_known, bool full_cave_known)
    : SimulationScenario("table_with_box_scenario.json"),
      name(std::string("Table_with_Box_") + "table_" + (table_known ? "" : "un") + "known_" + "visible_cave_" +
           (visible_cave_known ? "" : "un") + "known_" + "full_cave_" + (full_cave_known ? "" : "un") + "known") {
  Object table = getTable();
  Object known_cave = getVisibleCave();
  Object cave_back = getCaveBack();

  table_known ? known_obstacles.add(table) : unknown_obstacles.add(table);
  // visible_cave_known ? known_obstacles.add(known_cave) : unknown_obstacles.add(known_cave);
  // full_cave_known ? known_obstacles.add(cave_back) : unknown_obstacles.add(cave_back);

  combineObstacles();

  setPrior(unknown_obstacles, bp);

  // s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
  // s.current_config = VictorRightArmConfig(std::vector<double>{-1.5,1.5,-1.5,-0.4,-1.5,-1.0,1.5}).asMap();
  //    s.current_config = VictorRightArmConfig(std::vector<double>{-1.0, 1.5, -1.5, 0.4, -1.5, 0.0, 1.5}).asMap();
  //    goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();
}

void TableWithBox::setCaveDims() {
  cavecorner = Vector3f(1.7, 2.0, 0.9);
  caveheight = Vector3f(0.0, 0.0, 0.4);
  cavetopd = Vector3f(0.35, 0.5, 0.033);
  cavesidedim = Vector3f(0.033, cavetopd.y, caveheight.z);
  cavesideoffset = Vector3f(cavetopd.x, 0.0, 0.0);
}

Object TableWithBox::getTable() {
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

Object TableWithBox::getVisibleCave() {
  setCaveDims();
  Object cave;
  cave.add(AABB(cavecorner + caveheight, cavecorner + caveheight + cavetopd + Vector3f(0.033, 0, 0)));  // top
  cave.add(AABB(cavecorner, cavecorner + cavesidedim));
  return cave;
}

Object TableWithBox::getCaveBack() {
  setCaveDims();
  Object cave_back;
  cave_back.add(AABB(cavecorner + cavesideoffset, cavecorner + cavesideoffset + cavesidedim));
  return cave_back;
}

/****************************************
 **         SlottedWall
 ****************************************/
SlottedWall::SlottedWall(const BeliefParams &bp)
    : SimulationScenario("slotted_wall_scenario.json"), name("Slotted Wall") {
  bool all_known = (bp.belief_type == BeliefType::Deterministic);

  known_obstacles.add(getFrontWall());

  if (all_known) {
    known_obstacles.add(getSlottedWall());
  } else {
    unknown_obstacles.add(getSlottedWall());
  }

  combineObstacles();
  setPrior(unknown_obstacles, bp);

  // s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
  // goal_config = VictorRightArmConfig(std::vector<double>{0, 0.32, 0, -1.32, -0.2, 0.9, 0.3}).asMap();

  //    s.current_config = VictorRightArmConfig(std::vector<double>{-1.5, 1.0, -0.5, -0.5, 0, 0, 0}).asMap();
  //    goal_config = VictorRightArmConfig(std::vector<double>{0, 0.72, -0.3, -1.32, -1.2, 0.9, 0.3}).asMap();

  victor.set(s.current_config);
}

Object SlottedWall::getFrontWall() {
  Object front_wall;
  float lower_wall_height = 1.1;
  float gap_height = .4;

  Vector3f lfwc(1.5, 1.6, 0.0);  // lower front wall corner
  Vector3f lfwd(0.04, 0.4, lower_wall_height);

  Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height);  // upper front wall
  Vector3f ufwd(0.04, 0.4, 0.3);

  Vector3f mfwc(1.5, 1.8, 0);  // middle front wall
  Vector3f mfwd(0.04, 0.2, 1.5);

  front_wall.add(AABB(lfwc, lfwc + lfwd));
  front_wall.add(AABB(ufwc, ufwc + ufwd));
  front_wall.add(AABB(mfwc, mfwc + mfwd));
  return front_wall;
}

Object SlottedWall::getSlottedWall() {
  Object slotted_wall;
  float lower_wall_height = 1.1;
  float gap_height = .4;

  Vector3f lswc(1.5, 1.6, 0.0);                             // lower side wall corner
  Vector3f lswd(.75, 0.04, lower_wall_height);              // lower side wall dims
  Vector3f cswc(1.5, 1.6, lower_wall_height + gap_height);  // close side wall corner
  // Vector3f cswd(0.2, 0.04, 0.3);
  Vector3f cswd(0.5, 0.04, 0.3);
  Vector3f fswc(1.95, 1.6, lower_wall_height);  // far side wall corner
  Vector3f fswd(0.3, 0.04, 0.6);
  Vector3f mswc(1.95, 1.6, lower_wall_height + gap_height + .1f);  //
  Vector3f mswd(0.3, 0.04, 0.2);

  slotted_wall.add(AABB(lswc, lswc + lswd));  // lower side wall
  slotted_wall.add(AABB(cswc, cswc + cswd));  // close side wall
  slotted_wall.add(AABB(fswc, fswc + fswd));  // far side wall
  slotted_wall.add(AABB(mswc, mswc + mswd));  // middle side wall
  return slotted_wall;
}

/****************************************
 **         Bookshelf
 ****************************************/

Bookshelf::Bookshelf(const BeliefParams &bp)
    : SimulationScenario("bookshelf_scenario.json"), name(std::string("Bookshelf")) {
  robot::JointValueMap jvm;
  jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
  jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
  jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
  // ri.setRightGripper(0);
  victor.set(jvm);

  Object bookshelf = getBookshelf();
  Object table = getTable();

  known_obstacles.add(bookshelf);

  bool table_known = (bp.belief_type == BeliefType::Deterministic);
  if (table_known) {
    known_obstacles.add(table);
  } else {
    unknown_obstacles.add(table);
  }

  combineObstacles();
  setPrior(unknown_obstacles, bp);

  // s.current_config = VictorRightArmConfig(std::vector<double>
  //                                         {-0.9, 1.3, -0.3, -0.8, 0.0, 0.2, 0.3}).asMap();
  //    s.current_config = VictorRightArmConfig(std::vector<double>{-1.2, 1.3, -0.8, 0.4, 0.4, 0.3, 0.3}).asMap();
  //    goal_config = VictorRightArmConfig(std::vector<double>{0.3, 1.2, -0.3, 1.5, 0, -0.7, -0.9}).asMap();

  victor.set(s.current_config);
}

Object Bookshelf::getBookshelf() {
  float lower_wall_height = 1.1;
  float gap_height = .4;
  float bookshelf_width = 0.8;
  float bookshelf_height = 1.6;
  float bookshelf_depth = 0.4;
  Vector3f backwallc(1.2, 0.8, 0.0);  // backwall cornder
  Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
  Vector3f sidewall(0.04, bookshelf_depth, bookshelf_height);
  Vector3f swoff(bookshelf_width, 0, 0);
  Vector3f shelf(bookshelf_width + 0.04f, bookshelf_depth, 0.02);
  Vector3f shelf_spacing(0, 0, 0.4);

  Vector3f bookc(1.6, 0.82, 1.2);
  Vector3f bookd(0.05, 0.3, 0.3);

  Object bookshelf;

  bookshelf.add(AABB(backwallc, backwallc + backwall_thickness));
  bookshelf.add(AABB(backwallc, backwallc + sidewall));
  bookshelf.add(AABB(backwallc + swoff, backwallc + sidewall + swoff));
  for (int i = 0; i < 5; i++) {
    bookshelf.add(AABB(backwallc + shelf_spacing * (float)i, backwallc + shelf + shelf_spacing * (float)i));
  }
  bookshelf.add(AABB(bookc, bookc + bookd));

  return bookshelf;
}

Object Bookshelf::getTable() {
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tcf(1.7, 1.4, 0.0);                              // table corner at floor
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  Object table;
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
 **         CloseWall
 ****************************************/

CloseWall::CloseWall(const BeliefParams &bp)
    : SimulationScenario("close_wall_scenario.json"), name(std::string("CloseWall")) {
  robot::JointValueMap jvm;
  jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
  jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
  jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
  victor.set(jvm);

  Object wall = getCloseWall();

  bool wall_known = (bp.belief_type == BeliefType::Deterministic);
  if (wall_known) {
    known_obstacles.add(wall);
  } else {
    unknown_obstacles.add(wall);
  }

  combineObstacles();
  setPrior(unknown_obstacles, bp);

  // s.current_config = VictorRightArmConfig(std::vector<double>
  //                                         {-0.9, 1.3, -0.3, -0.8, 0.0, 0.2, 0.3}).asMap();
  //    s.current_config = VictorRightArmConfig(std::vector<double>{-0.5, 1.2, -1.5, 0.4, -1.5, 0.0, 1.5}).asMap();
  //    goal_config = VictorRightArmConfig(std::vector<double>{-0.0, -0.4, -1.5, -0.4, -1.5, -1.0, 1.5}).asMap();

  victor.set(s.current_config);
}

Object CloseWall::getCloseWall() {
  float bookshelf_width = 0.8;
  float bookshelf_height = 1.6;
  float bookshelf_depth = 0.4;
  Vector3f backwallc(1.6, 1.2, 0.0);  // backwall cornder
  Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
  Vector3f sidewall(0.04, bookshelf_depth, bookshelf_height);
  Vector3f swoff(bookshelf_width, 0, 0);
  Vector3f shelf(bookshelf_width + 0.04f, bookshelf_depth, 0.02);
  Vector3f shelf_spacing(0, 0, 0.4);

  Object wall;

  wall.add(AABB(backwallc, backwallc + backwall_thickness));
  wall.add(AABB(backwallc, backwallc + sidewall));
  wall.add(AABB(backwallc + swoff, backwallc + sidewall + swoff));
  for (int i = 0; i < 5; i++) {
    wall.add(AABB(backwallc + shelf_spacing * (float)i, backwallc + shelf + shelf_spacing * (float)i));
  }

  return wall;
}

/****************************************
**      ShapeRequest Scenario
****************************************/
ShapeRequestScenario::ShapeRequestScenario(const BeliefParams &bp)
    : SimulationScenario("shape_request_scenario.json"), name(std::string("ShapeRequestScenario")) {
  Object table = getObstacles();

  unknown_obstacles.add(table);
  // visible_cave_known ? known_obstacles.add(known_cave) : unknown_obstacles.add(known_cave);
  // full_cave_known ? known_obstacles.add(cave_back) : unknown_obstacles.add(cave_back);

  combineObstacles();

  setPrior(unknown_obstacles, bp);
}

Object ShapeRequestScenario::getObstacles() {
  Object obj;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gpu_voxel_planning_msgs::RequestShape>("/get_shape");
  gpu_voxel_planning_msgs::RequestShape srv;
  if (client.call(srv)) {
    std::cout << "Got shape\n";
    obj.occupied.insertPointCloud(toPointsVector(srv.response.points), PROB_OCCUPIED);
  } else {
    std::cout << "Service call failed\n";
  }
  return obj;
}

std::vector<robot::JointValueMap> ShapeRequestScenario::getPossibleGoals() const {
  //    if(known_goal_config.has_value()){
  //        return std::vector<robot::JointValueMap>{known_goal_config.value()};
  //    }

  return s.bel->getPossibleGoals(this);
  // TODO: Made more general for more types of beliefs
//  auto bel = dynamic_cast<ShapeCompletionBelief *>(s.bel.get());
//  std::vector<robot::JointValueMap> goal_configs;


  //  return s.bel->getPossibleGoals();
  //    throw std::runtime_error("Shape Request Scenario does not have a known_goal_config");
//  return goal_configs;
}

bool ShapeRequestScenario::completed() const {
  s.bel->syncBelief();
  auto bel = dynamic_cast<ShapeCompletionBelief *>(s.bel.get());
  auto cur_angles = VictorRightArmConfig(s.current_config).asVector();
  auto cur_pose = jacobian_follower.computeFK(cur_angles, "right_arm");

  int num_valid = 0;
  for (const auto &goal : bel->goal_tsrs) {
    // TODO Check orientation as well as position
    //    ????
//    auto tsrgoal = dynamic_cast<TSRGoal*>(goal.get());
//    if (tsrgoal->goal.contains(cur_pose)) {
    if(goal->isAchieved(VictorRightArmConfig(s.current_config), this)){
      num_valid += 1;
    }
  }

  double prob_complete = (double)num_valid / bel->goal_tsrs.size();
  std::cout << "cur pose is within " << prob_complete*100 << "% of the tsrs\n";
  return prob_complete >= 0.9;
}
