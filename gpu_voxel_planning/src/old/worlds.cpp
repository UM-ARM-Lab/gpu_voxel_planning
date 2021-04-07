#include "worlds.hpp"

#include <arm_pointcloud_utilities/load_save_to_file.h>
#include <gpu_voxel_planning_msgs/AttemptPathResult.h>
#include <gpu_voxel_planning_msgs/AttemptPathStart.h>
#include <gpu_voxel_planning_msgs/CollisionInformation.h>

#include "gpu_voxel_planning/common_names.hpp"
#include "gpu_voxel_planning/hardcoded_params.h"

using namespace arm_pointcloud_utilities;
using namespace PathUtils;

/*************************************
 **       SIM OBSTACLES             **
 ************************************/
SimWorld::SimWorld() {
  std::cout << "Creating sim world\n";
  gvl = gpu_voxels::GpuVoxels::getInstance();

  // if(USE_KNOWN_OBSTACLES)
  // {
  gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
  // }

  gvl->visualizeMap(SIM_OBSTACLES_MAP);

  double init_angles[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  init_config = victor_model.toVictorConfig(init_angles);
}

void SimWorld::initializeObstacles() { initializeVictor(); }

void SimWorld::initializeVictor() {
  VictorConfig left_arm_config;

  std::vector<double> left_arm_joint_values = {1.57, 1.57, 0, 0, 0, 0, 0};
  // std::vector<double> left_arm_joint_values = {-1.417, 1.566, -1.151, 1.293, 2.437, 1.406, -1.12};

  for (size_t i = 0; i < left_arm_joint_values.size(); i++) {
    left_arm_config[victor_model.left_arm_joint_names[i]] = left_arm_joint_values[i];
  }
  gvl->setRobotConfiguration(VICTOR_ROBOT_STATIONARY, left_arm_config);
  gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED);

  gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, SIM_OBSTACLES_MAP, PROB_OCCUPIED);

  VictorConfig right_gripper_config;
  // right_gripper_config["victor_right_gripper_fingerA_joint_2"] = 0;
  // right_gripper_config["victor_right_gripper_fingerB_joint_2"] = 0;
  // right_gripper_config["victor_right_gripper_fingerC_joint_2"] = 0;
  right_gripper_config["victor_right_gripper_fingerA_joint_2"] = 1.5;
  right_gripper_config["victor_right_gripper_fingerB_joint_2"] = 1.5;
  right_gripper_config["victor_right_gripper_fingerC_joint_2"] = 1.5;

  gvl->setRobotConfiguration(VICTOR_ROBOT, right_gripper_config);

  victor_model.updateActual(init_config);
}

/*
 *  Returns the single link in collision
 */

Maybe::Maybe<std::string> SimWorld::getCollisionLink(const VictorConfig &c) {
  victor_model.resetQuery();
  victor_model.addQueryState(c);
  if (victor_model.countNumCollisions(SIM_OBSTACLES_MAP) == 0) {
    victor_model.resetQuery();
    return Maybe::Maybe<std::string>();
  }

  victor_model.resetQuery();
  for (auto &link_name : victor_model.right_arm_collision_link_names) {
    victor_model.addQueryLink(c, link_name);
    if (victor_model.countNumCollisions(SIM_OBSTACLES_MAP) > 0) {
      victor_model.resetQuery();
      return Maybe::Maybe<std::string>(link_name);
    }
  }
  std::cout << "Victor collides, but no links collide...\n";
  assert(false);
}

/*
 *  Returns the set of links that might be in collision (Adds full gripper to link 6)
 */
Maybe::Maybe<std::vector<std::string>> SimWorld::getCollisionLinks(const VictorConfig &c) {
  Maybe::Maybe<std::string> col_link = getCollisionLink(c);
  if (!col_link.Valid()) {
    return Maybe::Maybe<std::vector<std::string>>();
  }

  std::vector<std::string> collision_links;

  std::vector<std::string>::iterator iter =
      std::find(victor_model.right_arm_collision_link_names.begin(), victor_model.right_arm_collision_link_names.end(),
                col_link.Get());

  // Add all links after collision link
  while (iter != victor_model.right_arm_collision_link_names.end()) {
    collision_links.push_back(*iter);
    iter++;
  }

  // collision_links.push_back(col_link.Get());
  /*
   * If collision with gripper add full gripper
   */
  // if(std::find(victor_right_gripper_collision_names.begin(),
  //              victor_right_gripper_collision_names.end(),
  //              col_link.Get()) != victor_right_gripper_collision_names.end())
  // {
  //     for(auto link: victor_right_gripper_collision_names)
  //     {
  //         collision_links.push_back(link);
  //     }
  // }

  return Maybe::Maybe<std::vector<std::string>>(collision_links);
}

/*
 *  Executes path until completion or collision.
 */
bool SimWorld::executePath(const Path &path, size_t &last_valid, bool add_col_set) {
  if (VIDEO_VISUALIZE) {
    victor_model.resetQuery();
    gvl->visualizeMap(VICTOR_QUERY_MAP);
  }

  for (last_valid = 0; last_valid < path.size(); last_valid++) {
    std::vector<double> joint_angles = path[last_valid];
    VictorConfig c = victor_model.toVictorConfig(joint_angles.data());
    Maybe::Maybe<std::vector<std::string>> col_links = getCollisionLinks(c);
    if (col_links.Valid()) {
      if (add_col_set) {
        Path col_path(path.begin() + last_valid, path.end());
        // std::copy(path.begin() + last_valid, path.end(), col_path.begin());
        victor_model.addCHS(col_path, col_links.Get());

        // std::vector<VictorConfig> cs;
        // for(size_t j=last_valid; (j<last_valid + NUM_STEPS_FOR_ADDING_COLLISION) && j<path.size(); j++)
        // {
        //     cs.push_back(victor_model.toVictorConfig(path[j].data()));
        // }
        // victor_model.addCHS(cs, col_links.Get());
      }

      last_valid--;
      return false;
    }

    victor_model.updateActual(c);

    usleep(EXECUTION_DELAY_us / 2);
    victor_model.doVis();
    usleep(EXECUTION_DELAY_us / 2);
  }
  last_valid = path.size() - 1;
  // std::cout << "Path success\n";
  return true;
}

// Path densifyPath(const Path &path, int densify_factor)
// {
//     // std::cout << "densifying path\n";
//     Path dense_path;
//     double dt = 1.0/(double)densify_factor;
//     for(size_t i=0; i < (path.size()-1); i++)
//     {
//         for(int new_seg=0; new_seg < densify_factor; new_seg++)
//         {
//             const std::vector<double> &cur = path[i];
//             const std::vector<double> &next = path[i+1];
//             std::vector<double> interp;
//             for(size_t j=0; j<cur.size(); j++)
//             {
//                 interp.push_back(cur[j] + (next[j] - cur[j]) * new_seg *dt);
//             }

//             dense_path.push_back(interp);
//         }
//     }
//     dense_path.push_back(path[path.size()-1]);
//     for(int i=0; i<(int)dense_path.size()-1; i++)
//     {
//         std::cout << PathUtils::dist(dense_path[i], dense_path[i+1]) << "\n";
//     }

//     return dense_path;
// }

void SimWorld::executeAndReturn(const Path &path) {
  Path dense_path = PathUtils::densify(path, 0.01);
  size_t last_valid;
  executePath(dense_path, last_valid, true);
  Path backup;

  while (true) {
    backup.push_back(dense_path[last_valid]);
    if (last_valid == 0) {
      break;
    }
    last_valid--;
  }
  executePath(backup, last_valid, false);
}

bool SimWorld::attemptPath(const Path &path) {
  Path dense_path = PathUtils::densify(path, 0.01);
  size_t last_valid;
  if (executePath(dense_path, last_valid, true)) {
    return true;
  }

  Path backup;
  for (int i = 0; i < 3; i++) {
    backup.push_back(dense_path[last_valid]);
    if (last_valid == 0) {
      break;
    }
    last_valid--;
  }
  executePath(backup, last_valid, false);
  return false;
}

/***
 **   Simulated Table World
 ***/
SimTable::SimTable() {
  // double init_angles[] = {-1.4, 1.4, 1.4, -0.5, 0.01, 0.01, 0.05};
  double init_angles[] = {0, 0, 0, 0, 0, 0, 0};
  double goal_angles[] = {-0.15, 1.0, 0, -0.5, 0, 1.0, 0};
  // double goal_angles[] = {0, 0.32, 0, -1.32, -0.2, 0.9, 0.3};
  init_config = victor_model.toVictorConfig(init_angles);
  goal_config = victor_model.toVictorConfig(goal_angles);

  initializeObstacles();
  initializeVictor();
}

void SimTable::initializeObstacles() { makeTable(); }

void SimTable::makeTable() {
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  // table top
  gvl->insertBoxIntoMap(tc, tc + td, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0), Vector3f(tc.x, tc.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP,
                        PROB_OCCUPIED, 2);

  Vector3f cavecorner;
  if (PEG_IN_HOLE) {
    cavecorner = Vector3f(1.7, 1.7, 0.9);
  } else {
    cavecorner = Vector3f(1.7, 2.0, 0.9);
  }
  Vector3f caveheight(0.0, 0.0, 0.4);
  Vector3f cavetopd(0.3, 0.5, 0.033);
  Vector3f cavesidedim(0.033, cavetopd.y, caveheight.z);
  Vector3f cavesideoffset(cavetopd.x, 0.0, 0.0);
  Vector3f caveholecorner = cavecorner + cavesideoffset + Vector3f(0, 0.15, 0.15);
  Vector3f caveholesize(.04, .15, .15);

  gvl->insertBoxIntoMap(cavecorner + caveheight, cavecorner + caveheight + cavetopd, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(cavecorner, cavecorner + cavesidedim, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(cavecorner + cavesideoffset, cavecorner + cavesideoffset + cavesidedim, SIM_OBSTACLES_MAP,
                        PROB_OCCUPIED, 2);

  if (PEG_IN_HOLE) {
    gvl->insertBoxIntoMap(caveholecorner, caveholecorner + caveholesize, TMP_MAP, PROB_OCCUPIED, 2);
    victor_model.getMap(SIM_OBSTACLES_MAP)->subtract(victor_model.getMap(TMP_MAP));
  }

  if (USE_KNOWN_OBSTACLES) {
    gvl->insertBoxIntoMap(tc, tc + td - Vector3f(0, .405, 0), KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(cavecorner + caveheight, cavecorner + caveheight + cavetopd, KNOWN_OBSTACLES_MAP,
                          PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(cavecorner, cavecorner + cavesidedim, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

    if (ALL_OBSTACLES_KNOWN) {
      gvl->insertBoxIntoMap(tc, tc + td, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

      gvl->insertBoxIntoMap(cavecorner + cavesideoffset, cavecorner + cavesideoffset + cavesidedim, KNOWN_OBSTACLES_MAP,
                            PROB_OCCUPIED, 2);
    }

    gvl->visualizeMap(SIM_OBSTACLES_MAP);
    gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
  }
}

/***
 **   Simulated Bookshelf World
 ***/
SimBookshelf::SimBookshelf() {
  // double init_angles[] = {-1.4, 1.4, 1.4, -0.5, 0.01, 0.01, 0.05};
  double init_angles[] = {-1.2, 1.3, -0.8, 0.4, 0.4, 0.3, 0.3};
  double goal_angles[] = {0.3, 1.2, -0.3, 1.5, 0, -0.7, -0.9};
  // double goal_angles[] = {0, 0.32, 0, -1.32, -0.2, 0.9, 0.3};
  init_config = victor_model.toVictorConfig(init_angles);
  goal_config = victor_model.toVictorConfig(goal_angles);

  initializeObstacles();
  initializeVictor();
}

void SimBookshelf::initializeObstacles() {
  makeTable();
  makeBookshelf();
  gvl->visualizeMap(SIM_OBSTACLES_MAP);
  gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
}

void SimBookshelf::makeBookshelf() {
  double lower_wall_height = 1.1;
  double gap_height = .4;

  Vector3f lfwc(1.5, 1.6, 0.0);  // lower front wall corner
  Vector3f lfwd(0.04, 0.4, lower_wall_height);

  Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height);  // upper front wall
  Vector3f ufwd(0.04, 0.4, 0.3);

  Vector3f mfwc(1.5, 1.8, 0);  // middle front wall
  Vector3f mfwd(0.04, 0.2, 1.5);

  Vector3f lswc = lfwc;                         // lower side wall corner
  Vector3f lswd(.75, 0.04, lower_wall_height);  // lower side wall dims
  Vector3f cswc = ufwc;                         // close side wall corner
  Vector3f cswd(0.2, 0.04, 0.3);
  Vector3f fswc(1.95, 1.6, lower_wall_height);  // far side wall corner
  Vector3f fswd(0.3, 0.04, 0.6);
  Vector3f mswc(1.95, 1.6, lower_wall_height + gap_height + .1);  // far side wall corner
  Vector3f mswd(0.3, 0.04, 0.2);

  double bookshelf_width = 0.8;
  double bookshelf_height = 1.6;
  double bookshelf_depth = 0.4;
  Vector3f backwallc(1.2, 0.8, 0.0);  // backwall cornder
  Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
  Vector3f sidewall(0.04, bookshelf_depth, bookshelf_height);
  Vector3f swoff(bookshelf_width, 0, 0);
  Vector3f shelf(bookshelf_width + 0.04, bookshelf_depth, 0.02);
  Vector3f shelf_spacing(0, 0, 0.4);

  Vector3f bookc(1.6, 0.82, 1.2);
  Vector3f bookd(0.05, 0.3, 0.3);

  gvl->insertBoxIntoMap(backwallc, backwallc + backwall_thickness, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(backwallc, backwallc + sidewall, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(backwallc + swoff, backwallc + sidewall + swoff, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  for (int i = 0; i < 5; i++) {
    gvl->insertBoxIntoMap(backwallc + shelf_spacing * (float)i, backwallc + shelf + shelf_spacing * (float)i,
                          SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  }
  // gvl->insertBoxIntoMap(bookc, bookc+bookd));

  gvl->insertBoxIntoMap(backwallc, backwallc + backwall_thickness, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(backwallc, backwallc + sidewall, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(backwallc + swoff, backwallc + sidewall + swoff, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  for (int i = 0; i < 5; i++) {
    gvl->insertBoxIntoMap(backwallc + shelf_spacing * (float)i, backwallc + shelf + shelf_spacing * (float)i,
                          KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  }
}

void SimBookshelf::makeTable() {
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  // table top
  gvl->insertBoxIntoMap(tc, tc + td, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0), Vector3f(tc.x, tc.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP,
                        PROB_OCCUPIED, 2);

  // gvl->insertBoxIntoMap(tc, tc + td,
  //                       KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  // gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0),
  //                       Vector3f(tc.x, tc.y, 0) + tld,
  //                       KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  // gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
  //                       Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld,
  //                       KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  // gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
  //                       Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld,
  //                       KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  // gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
  //                       Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld,
  //                       KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
}

/***
 **   Simulated Table World
 ***/
SimEmptyTable::SimEmptyTable() {
  // double init_angles[] = {-1.4, 1.4, 1.4, -0.5, 0.01, 0.01, 0.05};
  double init_angles[] = {-0.5, 0.4, 0, -0.1, 0, 0, 0};
  double goal_angles[] = {-0.5, 1.5, 0.0, -0.7, 0.0, 0.0, 0.0};
  init_config = victor_model.toVictorConfig(init_angles);
  goal_config = victor_model.toVictorConfig(goal_angles);

  initializeObstacles();
  initializeVictor();
}

void SimEmptyTable::initializeObstacles() { makeTable(); }

void SimEmptyTable::makeTable() {
  Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254);  // table dimensions
  Vector3f tc(1.7, 1.4, 0.9);                               // table corner
  Vector3f tld(.033, 0.033, tc.z);                          // table leg dims

  // table top
  gvl->insertBoxIntoMap(tc, tc + td, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0), Vector3f(tc.x, tc.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, 0, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP, PROB_OCCUPIED,
                        2);
  gvl->insertBoxIntoMap(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x - tld.x, td.y - tld.y, 0) + tld, SIM_OBSTACLES_MAP,
                        PROB_OCCUPIED, 2);

  gvl->visualizeMap(SIM_OBSTACLES_MAP);
}

/*
 *   Simulated Solid Wall
 */
SimWall::SimWall() {
  double init_angles[] = {0, 0, 0, 0, 0, 0, 0};
  double goal_angles[] = {-0.15, 1.2, 0, -0.5, 0, 1.0, 0};
  init_config = victor_model.toVictorConfig(init_angles);
  goal_config = victor_model.toVictorConfig(goal_angles);

  initializeObstacles();
  initializeVictor();
}

void SimWall::initializeObstacles() {
  gvl->insertBoxIntoMap(Vector3f(1.6, 1.4, 0.5), Vector3f(3.0, 1.5, 1.5), SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->visualizeMap(SIM_OBSTACLES_MAP);
}

/***
 **   Simulated SlottedWall
 ***/
SimSlottedWall::SimSlottedWall() {
  initializeObstacles();
  initializeVictor();
}

void SimSlottedWall::initializeObstacles() { makeSlottedWall(); }

void SimSlottedWall::makeSlottedWall() {
  std::cout << "Making slotted walls\n";
  // Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 0.0 * 0.0254); //table dimensions
  // Vector3f tc(1.7, 1.4, 0.9); //table corner
  // Vector3f tld(.033, 0.033, tc.z); //table leg dims

  double lower_wall_height = 1.1;
  double gap_height = .4;

  Vector3f lfwc(1.5, 1.6, 0.0);  // lower front wall corner
  Vector3f lfwd(0.04, 1.5, lower_wall_height);

  Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height);  // upper front call
  Vector3f ufwd(0.04, 1.5, 0.3);

  Vector3f mfwc(1.5, 1.8, 0);  // middle front wall
  Vector3f mfwd(0.04, 1.4, 1.5);

  Vector3f lswc = lfwc;                         // lower side wall corner
  Vector3f lswd(1.5, 0.04, lower_wall_height);  // lower side wall dims
  Vector3f cswc = ufwc;                         // close side wall corner
  Vector3f cswd(0.2, 0.04, 0.3);
  Vector3f fswc(1.95, 1.6, lower_wall_height);  // far side wall corner
  Vector3f fswd(0.3, 0.04, 0.6);
  Vector3f mswc(1.95, 1.6, lower_wall_height + gap_height + .1);  // far side wall corner
  Vector3f mswd(0.3, 0.04, 0.2);

  gvl->insertBoxIntoMap(lfwc, lfwc + lfwd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(ufwc, ufwc + ufwd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(mfwc, mfwc + mfwd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);

  gvl->insertBoxIntoMap(lswc, lswc + lswd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(cswc, cswc + cswd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(fswc, fswc + fswd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);
  gvl->insertBoxIntoMap(mswc, mswc + mswd, SIM_OBSTACLES_MAP, PROB_OCCUPIED, 2);

  if (USE_KNOWN_OBSTACLES) {
    gvl->insertBoxIntoMap(lfwc, lfwc + lfwd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(lswc, lswc + lswd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);

    gvl->insertBoxIntoMap(ufwc, ufwc + ufwd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    gvl->insertBoxIntoMap(mfwc, mfwc + mfwd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    if (ALL_OBSTACLES_KNOWN) {
      gvl->insertBoxIntoMap(lswc, lswc + lswd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
      gvl->insertBoxIntoMap(cswc, cswc + cswd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
      gvl->insertBoxIntoMap(fswc, fswc + fswd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
      gvl->insertBoxIntoMap(mswc, mswc + mswd, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED, 2);
    }
  }
  gvl->visualizeMap(SIM_OBSTACLES_MAP);
  gvl->visualizeMap(KNOWN_OBSTACLES_MAP);
}

/***********************************
 **          REAL WORLD           **
 **********************************/
RealWorld::RealWorld() {
  std::cout << "Creating real world\n";
  gvl = gpu_voxels::GpuVoxels::getInstance();
  gvl->visualizeMap(KNOWN_OBSTACLES_MAP);

  ros::NodeHandle n;
  joint_sub = n.subscribe("/joint_states", 1, &RealWorld::jointStateCallback, this);
  attempt_path_client = n.serviceClient<gpu_voxel_planning_msgs::AttemptPathStart>("attempt_path_on_victor");
  get_attempt_status_client = n.serviceClient<gpu_voxel_planning_msgs::AttemptPathResult>("get_path_status");

  // loadPointCloudFromFile();
  double init_angles[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  VictorConfig init_config = victor_model.toVictorConfig(init_angles);
  victor_model.updateActual(init_config);

  update_victor_from_messages = true;
  pos_updated = false;
}

RealWorld::~RealWorld() {
  gvl.reset();
  victor_model.gvl.reset();
}

void RealWorld::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  VictorConfig cur;
  if (update_all_joint_count++ < 3) {
    for (size_t i = 0; i < msg->position.size(); i++) {
      cur[msg->name[i]] = msg->position[i];
    }
    gvl->setRobotConfiguration(VICTOR_ROBOT_STATIONARY, cur);
    gvl->insertRobotIntoMap(VICTOR_ROBOT_STATIONARY, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED);
  } else {
    std::vector<double> right_arm_angles(&msg->position[7], &msg->position[14]);
    cur = victor_model.toVictorConfig(right_arm_angles.data());
  }

  victor_model.updateActual(cur);
  victor_model.getMap(KNOWN_OBSTACLES_MAP)->subtract(victor_model.getMap(VICTOR_ACTUAL_MAP));
  victor_model.doVis();
  pos_updated = true;
}

void RealWorld::spinUntilUpdate() {
  pos_updated = false;
  while (ros::ok() && !pos_updated) {
    ros::Duration(0.001).sleep();
    ros::spinOnce();
  }
  gvl->visualizeMap(VICTOR_ACTUAL_MAP);
}

void RealWorld::loadPointCloudFromFile() {
  const std::pair<std::string, Eigen::Matrix3Xf> deserialized =
      LoadPointsetFromFile("./logs/point_cloud_latest.compressed");

  const Eigen::Matrix3Xf &mat = deserialized.second;
  std::cout << "Loading matrix of size " << mat.rows() << ", " << mat.cols() << "\n";
  std::vector<Vector3f> points;
  for (size_t i = 0; i < mat.cols(); i++) {
    if (std::isnan(mat(0, i)) || std::isnan(mat(1, i)) || std::isnan(mat(2, i))) {
      continue;
    }
    // std::cout << mat(0, i) << ", " << mat(1,i) << ", " << mat(2,i) << "\n";
    Vector3f p(1.0 + mat(0, i), 2.0 + mat(1, i), mat(2, i));

    // right arm
    if (p.x < 1.7 && p.y < 2.0 && p.z > 1.0) continue;

    // if(p.z > 0.5 && p.z < 1.05 &&
    //    p.y > 1.6)
    // {
    //     continue;
    // }

    // no point cloud
    continue;

    points.push_back(p);
  }
  gvl->insertPointCloudIntoMap(points, KNOWN_OBSTACLES_MAP, PROB_OCCUPIED);
}

bool RealWorld::attemptPath(const Path &path) {
  if (VIDEO_VISUALIZE) {
    victor_model.resetQuery();

    gvl->visualizeMap(VICTOR_QUERY_MAP);
  }

  gpu_voxel_planning_msgs::AttemptPathStart srv;
  srv.request.path.points.resize(path.size());
  for (size_t i = 0; i < path.size(); i++) {
    srv.request.path.points[i].positions = path[i];
  }

  if (attempt_path_client.call(srv)) {
    std::cout << "New path message sent\n";
  }

  gpu_voxel_planning_msgs::AttemptPathResult path_res;
  bool path_finished = false;
  while (ros::ok() && !path_finished) {
    // std::cout << "path not yet finished\n";
    get_attempt_status_client.call(path_res);
    path_finished = path_res.response.finished;
    spinUntilUpdate();
  }
  std::cout << "Path finished\n";

  if (path_res.response.ci.collided) {
    gpu_voxel_planning_msgs::CollisionInformation &ci = path_res.response.ci;
    std::cout << "Collision found on path\n";

    Path col_points;
    for (auto &traj_point : ci.collision_path.points) {
      col_points.push_back(traj_point.positions);
    }
    std::vector<std::string> col_links = ci.collision_links;
    col_links.insert(col_links.end(), victor_model.right_gripper_collision_link_names.begin(),
                     victor_model.right_gripper_collision_link_names.end());
    victor_model.addCHS(col_points, col_links);
    std::cout << "added " << col_points.size() << " collision configs\n";
    std::cout << "for links \n";
    for (auto &link_name : col_links) {
      std::cout << link_name << "\n";
    }
    spinUntilUpdate();

    return false;
  }

  return true;
}
