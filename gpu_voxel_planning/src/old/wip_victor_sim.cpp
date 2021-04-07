#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <arc_utilities/timing.hpp>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "gpu_voxel_planning/helpers.hpp"
#include "gpu_voxels_victor.hpp"
#include "victor_local_controller.hpp"
#include "victor_planning.hpp"
#include "worlds.hpp"

#define PROB_OCCUPIED eBVM_OCCUPIED

SimWorld* g_sim_world;

using namespace gpu_voxels_planner;
using namespace PathUtils;
namespace M = Maybe;

/***********************************************
 **                   MAIN                    **
 ***********************************************/

// // We define exit handlers to quit the program:
void ctrlchandler(int) {
  std::cout << "Resetting\n";
  g_sim_world->gvl.reset();
  exit(EXIT_SUCCESS);
}
void killhandler(int) {
  std::cout << "Resetting\n";
  g_sim_world->gvl.reset();
  exit(EXIT_SUCCESS);
}

bool attemptGoal(VictorPlanner& planner, std::vector<double> goal) {
  VictorConfig goal_config = g_sim_world->victor_model.toVictorConfig(goal.data());

  bool reached_goal = false;

  while (!reached_goal) {
    M::Maybe<Path> maybe_path = planner.planPathConfig(g_sim_world->victor_model.cur_config, goal_config);
    if (!maybe_path.Valid()) {
      std::cout << "Path planning failed\n";
      return false;
    }
    reached_goal = g_sim_world->attemptPath(maybe_path.Get());
  }
  return true;
}

void wiggleFingers() {
  VictorConfig right_gripper_config;

  while (true) {
    for (double p = -1.5; p < 1.5; p += 0.1) {
      right_gripper_config["victor_right_gripper_fingerA_joint_2"] = p;
      right_gripper_config["victor_right_gripper_fingerB_joint_2"] = p;
      right_gripper_config["victor_right_gripper_fingerC_joint_2"] = p;
      g_sim_world->victor_model.updateActual(right_gripper_config);
      g_sim_world->victor_model.doVis();
      usleep(100000);
      std::cout << "fingers at " << p << "\n";
    }
    for (double p = 1.5; p > -1.5; p -= 0.1) {
      right_gripper_config["victor_right_gripper_fingerA_joint_2"] = p;
      right_gripper_config["victor_right_gripper_fingerB_joint_2"] = p;
      right_gripper_config["victor_right_gripper_fingerC_joint_2"] = p;
      g_sim_world->victor_model.updateActual(right_gripper_config);
      g_sim_world->victor_model.doVis();
      usleep(100000);
      std::cout << "fingers at " << p << "\n";
    }
  }
}

void testAngles() {
  while (true) {
    std::string unused;
    std::cout << "Waiting for user input to set position...\n";
    std::getline(std::cin, unused);

    std::ifstream myfile;
    myfile.open(ros::package::getPath("gpu_voxel_planning") + "/config/test_angles.txt");
    std::cout << "file is open: " << myfile.is_open() << "\n";
    std::vector<double> goal_tmp;
    goal_tmp.resize(7);
    for (int i = 0; i < 7; i++) {
      myfile >> goal_tmp[i];
      std::cout << goal_tmp[i] << ", ";
    }
    std::cout << "\n";
    myfile.close();
    g_sim_world->victor_model.updateActual(g_sim_world->victor_model.toVictorConfig(goal_tmp.data()));
    g_sim_world->victor_model.doVis();
  }
}

void wip_SamplingRRTConnect(SimWorld* sim_world) {
  GpuVoxelsVictor& vm = sim_world->victor_model;
  // vm.gvl->insertBoxIntoMap(Vector3f(1.6, 1.4, 0.5), Vector3f(3.0 ,1.5,1.5),
  //                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);

  std::string unused;
  std::cout << "Waiting for user input to start...\n";
  std::getline(std::cin, unused);

  // double start[] = {-0.15,0,0,0,0,0,0};
  // double goal[] = {-0.15, 1.2, 0, -0.5, 0, 1.0, 0};
  // robot::JointValueMap sconfig = vm.toVictorConfig(start);

  robot::JointValueMap gconfig = sim_world->goal_config;

  VictorSamplingRRTConnect planner(&vm);
  VictorLocalController controller(&vm);
  bool reached_goal = false;
  while (!reached_goal) {
    robot::JointValueMap sconfig = vm.cur_config;
    // vm.sampleValidWorld();
    // std::cout << "Sampled world has " << vm.countVoxels(SAMPLED_WORLD_MAP) << " voxels\n";
    // std::cout << "Victor has " << vm.num_observed_chs << " chss\n";

    // Maybe::Maybe<Path> path = planner.planPathConfig(sconfig, gconfig);
    Maybe::Maybe<Path> path = planner.planPathConfig(sconfig, gconfig);
    reached_goal = sim_world->attemptPath(path.Get());

    for (int i = 0; i < 20; i++) {
      sconfig = vm.cur_config;
      sim_world->attemptPath(controller.maxExpectedChsIG(vm.toValues(sconfig), 0.1, 40));
      // vm.gvl->visualizeMap(VICTOR_QUERY_MAP);
      waitForKeypress();
    }
  }
}

void wip_Diverse(SimWorld* sim_world) {
  GpuVoxelsVictor& vm = sim_world->victor_model;
  std::string unused;
  std::cout << "Waiting for user input to start...\n";
  std::getline(std::cin, unused);
  robot::JointValueMap gconfig = sim_world->goal_config;

  DiversePlanner planner(&vm);
  bool reached_goal = false;
  arc_utilities::Stopwatch stopwatch;
  double allowed_time = 60 * 4;
  while (!reached_goal && stopwatch() < allowed_time) {
    robot::JointValueMap sconfig = vm.cur_config;
    Maybe::Maybe<Path> path = planner.planPathConfig(sconfig, gconfig);
    if (path.Valid()) {
      reached_goal = sim_world->attemptPath(path.Get());
    }
  }
}

int main(int argc, char* argv[]) {
  icl_core::logging::initialize(argc, argv);

  g_sim_world = new SimTable;
  // g_sim_world = new SimWall;
  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  // wip_SamplingRRTConnect(g_sim_world);
  // wip_Diverse(g_sim_world);

  // std::string unused;
  // std::cout << "Waiting for user input to start...\n";
  // std::getline(std::cin, unused);

  // wiggleFingers();
  // testAngles();

  // VictorLBKPiece planner(&(sim_world->victor_model));
  // VictorThresholdRRTConnect planner(&(sim_world->victor_model));
  VictorProbColCostRRTConnect planner(&(g_sim_world->victor_model));

  // VictorPRM planner(&(sim_world->victor_model));
  // VictorLazyRRTF planner(&(sim_world->victor_model));

  // attemptGoal(planner, goal);

  // M::Maybe<Path> maybe_path = planner.planPathDouble(start, goal);

  // if(!maybe_path.Valid())
  // {
  //     std::cout << "no path found\n";
  //     return 0;
  // }
  // std::cout << "Path found\n";
  // Path path = maybe_path.Get();
  // sim_world->attemptPath(path);

  g_sim_world->gvl.reset();
}
