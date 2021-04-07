#include "victor_planning.hpp"

#include "gpu_voxel_planning/hacky_functions.hpp"
#include "gpu_voxel_planning/ompl_utils.hpp"

#define ENABLE_PROFILING
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <ompl/geometric/PathSimplifier.h>
#include <signal.h>
#include <stdlib.h>

#include <arc_utilities/timing.hpp>
#include <iostream>
#include <memory>

#include "custom_rrtconnect.h"
#include "gpu_voxel_planning/hardcoded_params.h"
#include "gpu_voxel_planning/path_smoothing.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;
using namespace ompl_utils;
using namespace PathUtils;

const std::string FULL_PLANNING = "Full Planning";
const std::string PLANNING = "unsmoothed planning";
const std::string POST_PROCESSING = "post processing";
const std::string SMOOTHING = "smoothing";
const std::string VISUALIZE_SOLUTION = "visualize solution";

const std::string INSERT_VIZ_SOLUTION = "insert into viz solution";

const std::string ISVALID_INSERTION = "isValid insertion";
const std::string QUERY_INSERTION = "query insertion";
const std::string ISVALID_COLLISION_TEST = "isValid collision test";
const std::string CHECK_MOTION_SIMPLE_INSERTION = "checkMotion (simple) insertion";
const std::string CHECK_MOTION_SIMPLE_COLLISION_TEST = "checkMotion (simple) collision test";
const std::string CHECK_MOTION_SIMPLE_CHECK = "checkMotion (simple) full check";
const std::string CHECK_MOTION_COMP_CHECK = "checkMotion (complicated) full check";

const std::string STATE_COST = "stateCost";
const std::string MOTION_COST = "motionCost";

VictorPlanner::VictorPlanner(GpuVoxelsVictor* victor_model) {
  space = std::make_shared<ob::RealVectorStateSpace>();

  // ob::RealVectorBounds bounds(7);
  // bounds.setLow(-3.14159265);
  // bounds.setHigh(3.14159265);
  // space->setBounds(bounds);

  for (int i = 0; i < 7; i++) {
    space->addDimension(right_joint_lower_deg[i] * torad, right_joint_upper_deg[i] * torad);
  }

  // space->setLongestValidSegmentFraction(0.01);

  si_ = std::make_shared<ob::SpaceInformation>(space);
  // std::cout << "Original resolution " << si_->getStateValidityCheckingResolution();
  // si_->setStateValidityCheckingResolution(0.001);
  // vv_ptr = std::shared_ptr<VictorValidator>(std::make_shared<VictorValidator>(si_, victor_model));
  simp_ = std::make_shared<og::PathSimplifier>(si_);
  victor_model_ = victor_model;
}

void VictorPlanner::setupSpaceInformation() {
  std::cout << "using original setupSpaceInformation\n";
  si_->setStateValidityChecker(vv_ptr->getptr());
  si_->setMotionValidator(vv_ptr->getptr());
  si_->setup();
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::planPath(ob::ScopedState<> start, Goals goals) {
  preparePlanner(start, goals);
  ob::PlannerStatus solved = planner_->solve(10);

  while (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
    std::cout << "Approximate solution, replanning\n";
    solved = planner_->solve(3 * 10);
  }

  std::cout << solved << "\n";
  ob::PathPtr path;
  // If a solution has been found, we simplify and display it.
  if (solved) {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    path = pdef_->getSolutionPath();
    // std::cout << "Found solution. Simplifying..." << std::endl;
    // print the path to screen
    // path->print(std::cout);

    PROFILE_START(SMOOTHING);
    // simp_->simplifyMax(*(path->as<og::PathGeometric>()));
    simp_->shortcutPath(*(path->as<og::PathGeometric>()), 200, 200);
    PROFILE_RECORD(SMOOTHING);

    // std::cout << "Simplified solution:" << std::endl;
    // print the path to screen
    // path->print(std::cout);
    PROFILE_START(VISUALIZE_SOLUTION);

    // victor_model->visualizeSolution(path);

    PROFILE_RECORD(VISUALIZE_SOLUTION);

    std::cout << "Calling post planning actions\n";
    post_planning_actions(path);

  } else {
    std::cout << "No solution could be found" << std::endl;
    return Maybe::Maybe<ob::PathPtr>();
  }

  // PROFILE_RECORD(POST_PROCESSING);
  // PROFILE_RECORD(FULL_PLANNING);

  // std::vector<std::string> summary_names = {
  //     FULL_PLANNING,
  //     "~~~~~~~~~~~~",
  //     PLANNING,
  //     POST_PROCESSING,
  //     "~~~~~~~~~~~~",
  //     SMOOTHING,
  //     VISUALIZE_SOLUTION,
  //     INSERT_VIZ_SOLUTION,
  //     "~~~~~~~~~~~~",
  //     ISVALID_INSERTION,
  //     ISVALID_COLLISION_TEST,
  //     "~~~~~~~~~~~~",
  //     CHECK_MOTION_SIMPLE_CHECK,
  //     CHECK_MOTION_SIMPLE_INSERTION,
  //     CHECK_MOTION_SIMPLE_COLLISION_TEST,
  //     "~~~~~~~~~~~~",
  //     CHECK_MOTION_COMP_CHECK,
  //     "~~~~~~~~~",
  //     STATE_COST,
  //     MOTION_COST};

  // PROFILE_PRINT_SUMMARY_FOR_GROUP(summary_names);

  return Maybe::Maybe<ob::PathPtr>(path);
}

Maybe::Maybe<Path> VictorPlanner::planPathConfig(VictorConfig start, VictorConfig goals) {
  return planPathDouble(victor_model_->toValues(start), victor_model_->toValues(goals));
}

Maybe::Maybe<Path> VictorPlanner::planPathDouble(std::vector<double> start, std::vector<double> goal) {
  ob::ScopedState<> start_ss(space);
  ob::ScopedState<> goal_ss(space);
  std::cout << "start: ";
  start_ss = toScopedState(start, space);
  goal_ss = toScopedState(goal, space);

  Maybe::Maybe<ob::PathPtr> ompl_path = planPath(start_ss, goal_ss);
  if (!ompl_path.Valid()) {
    return Maybe::Maybe<Path>();
  }
  return Maybe::Maybe<Path>(omplPathToDoublePath(ompl_path.Get()->as<og::PathGeometric>(), si_));
}

VictorConfig VictorPlanner::toVictorConfig(ob::ScopedState<> s) {
  return victor_model_->toVictorConfig(s->as<ob::RealVectorStateSpace::StateType>()->values);
}

Path VictorPlanner::iouWiggleConfig(VictorConfig start) {
  ob::PathPtr ompl_path = iouWiggle(toScopedState(victor_model_->toValues(start), space));
  return omplPathToDoublePath(ompl_path->as<og::PathGeometric>(), si_);
}

ob::PathPtr VictorPlanner::iouWiggle(ob::ScopedState<> start) {
  double min_iou = 1.1;
  ob::ScopedState<> best_state(space);

  for (int i = 0; i < 50; i++) {
    double max_dist = space->getLongestValidSegmentLength() * 2;
    ob::ScopedState<> new_state = samplePointInRandomDirection(start, si_, max_dist);

    Ompl_Path ompl_path(std::make_shared<og::PathGeometric>(si_));
    ompl_path->append(start.get());
    ompl_path->append(new_state.get());
    ompl_path->interpolate();

    Path path = PathUtils::densify(omplPathToDoublePath(ompl_path.get(), si_), 0.01);

    double iou = evaluateIouExploration(path);

    if (iou < min_iou) {
      min_iou = iou;
      best_state = new_state;
    }

    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    std::string unused;
    std::getline(std::cin, unused);
  }

  auto path(std::make_shared<og::PathGeometric>(si_));
  path->append(start.get());
  path->append(best_state.get());
  return path;
}

/*
 *  Returns the expected IoU compared to the most similar collision hypothesis set
 */
double VictorPlanner::evaluateIouExploration(Path path) {
  std::vector<size_t> seen_sizes = victor_model_->chsSizes();
  victor_model_->resetQuery();

  for (auto jvs : path) {
    victor_model_->addQueryState(victor_model_->toVictorConfig(jvs.data()));
  }

  victor_model_->removeSweptVolume(VICTOR_QUERY_MAP);

  size_t query_size = victor_model_->countIntersect(VICTOR_QUERY_MAP, FULL_MAP);
  std::vector<size_t> intersections = victor_model_->countCHSCollisions();

  double p_no_col = 1.0;
  for (size_t i = 0; i < seen_sizes.size(); i++) {
    p_no_col *= 1 - (double)intersections[i] / (double)seen_sizes[i];
  }
  double p_col = 1.0 - p_no_col;

  double max_iou_if_col = 0;
  for (size_t i = 0; i < seen_sizes.size(); i++) {
    double iou_if_col = (double)intersections[i] / (double)(query_size + seen_sizes[i] - intersections[i]);
    max_iou_if_col = std::max(iou_if_col, max_iou_if_col);
  }

  double max_iou_if_no_col = 0;
  for (size_t i = 0; i < seen_sizes.size(); i++) {
    double iou_if_no_col = (double)(seen_sizes[i] - intersections[i]) / (double)(seen_sizes[i]);
    max_iou_if_no_col = std::max(iou_if_no_col, max_iou_if_no_col);
  }

  double iou = p_col * max_iou_if_col + (1 - p_col) * max_iou_if_no_col;
  std::cout << "Iou: " << iou << "\n";
  std::cout << "pcol: " << p_col << "\n";
  return iou;
}

Path VictorPlanner::randomWiggleConfig(VictorConfig start) {
  ob::PathPtr ompl_path = randomWiggle(toScopedState(victor_model_->toValues(start), space));
  return omplPathToDoublePath(ompl_path->as<og::PathGeometric>(), si_);
}

ob::PathPtr VictorPlanner::randomWiggle(ob::ScopedState<> start) {
  double max_dist = space->getLongestValidSegmentLength() * 2;
  ob::ScopedState<> new_state = samplePointInRandomDirection(start, si_, max_dist);

  auto path(std::make_shared<og::PathGeometric>(si_));
  path->append(start.get());
  path->append(new_state.get());
  return path;
}

Maybe::Maybe<Path> VictorPlanner::localControlConfig(VictorConfig start, VictorConfig goal) {
  return localControlDouble(victor_model_->toValues(start), victor_model_->toValues(goal));
}

Maybe::Maybe<Path> VictorPlanner::localControlDouble(std::vector<double> start, std::vector<double> goal) {
  Maybe::Maybe<ob::PathPtr> ompl_path = localControl(toScopedState(start, space), toScopedState(goal, space));
  if (!ompl_path.Valid()) {
    return Maybe::Maybe<Path>();
  }
  return Maybe::Maybe<Path>(omplPathToDoublePath(ompl_path.Get()->as<og::PathGeometric>(), si_));
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::localControl(ob::ScopedState<> start, Goals goals) {
  // std::cout << "Using probcol local control\n";
  if (vppc.get() == nullptr) {
    std::cout << "Cannot run local controller without allocated rplanner\n";
    return Maybe::Maybe<ob::PathPtr>();
  }

  int num_samples = 20;
  int i = 0;

  double start_d_to_goal = goals.distance(start);

  double max_motion = space->getLongestValidSegmentLength() * 2;

  ob::ScopedState<> test(space);
  ob::State* best(si_->allocState());
  double best_ev = 0;
  bool found_ok = false;

  vppc->setProbabilityThreshold(std::numeric_limits<double>::max());
  double best_pcol = std::numeric_limits<double>::max();

  double goal_sampled = false;

  bool checking_goal_directly = false;

  // while(i < num_samples)
  for (i = 0; i < num_samples; i++) {
    checking_goal_directly = (i == 0);
    bool near_goal = goals.distance(start) <= max_motion;
    if (checking_goal_directly) {
      si_->copyState(test.get(), goals.get());
      goal_sampled = true;
    } else {
      double max_dist = space->getLongestValidSegmentLength() * 2;
      test = samplePointInRandomDirection(start, si_, max_dist);
    }

    double progress_d = start_d_to_goal - goals.distance(test.get());

    if (progress_d <= 0)  // || progress_d <= best_ev)
    {
      if (i > 2 && !near_goal) {
        i--;
      }
      continue;
    }

    size_t num_interp_points = 2;
    std::vector<ob::State*> dpath;
    si_->getMotionStates(start.get(), test.get(), dpath, num_interp_points, true, true);
    size_t col_index;
    double pcol = vppc->getPathCost(dpath, col_index);
    si_->freeStates(dpath);

    bool debug = false;
    if (debug) {
      std::cout << "pcol " << pcol << "\n ";
      victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
      std::string unused;
      std::getline(std::cin, unused);
    }

    if (pcol > 0.9) {
      continue;
    }

    // double ev = (1.0 - pcol) * (progress_d);
    double ev = (1.0 - pcol);
    if (ev > best_ev) {
      si_->copyState(best, test.get());
      best_ev = ev;
      found_ok = true;
      best_pcol = pcol;
    }

    if (checking_goal_directly && pcol < .2) {
      // good enough to go towards goal
      break;
    }

    // if(checking_goal_directly)
    // {
    //     std::cout << "Direct path towards goal has pcol " << pcol << "\n";
    //     if(pcol < .1)
    //     {
    //         break;
    //     }
    // }
  }

  if (!found_ok) {
    return Maybe::Maybe<ob::PathPtr>();
  }

  bool debug = false;
  if (debug) {
    std::vector<ob::State*> dpath;
    dpath.push_back(start.get());
    dpath.push_back(best);

    size_t col_index;
    std::cout << "best pcol " << best_pcol << ", when reevaluated ";
    std::cout << vppc->getPathCost(dpath, col_index) << "\n";

    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    std::string unused;
    std::getline(std::cin, unused);
  }

  if (debug && goal_sampled) {
    std::vector<ob::State*> dpath;
    dpath.push_back(start.get());
    dpath.push_back(best);

    size_t col_index;
    std::cout << "best pcol " << best_pcol << ", when reevaluated ";
    std::cout << vppc->getPathCost(dpath, col_index) << "\n";
    std::cout << "distance from goal " << goals.distance(best) << "\n";

    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    std::string unused;
    std::getline(std::cin, unused);
  }

  auto path(std::make_shared<og::PathGeometric>(si_));
  path->append(start.get());
  path->append(best);

  return Maybe::Maybe<ob::PathPtr>(path);
}

Maybe::Maybe<ob::PathPtr> VictorPlanner::voxLocalControl(ompl::base::ScopedState<> start, Goals goals) {
  // std::cout << "Using vox local control\n";
  if (vppc.get() == nullptr) {
    std::cout << "Cannot run local controller without allocated rplanner\n";
    return Maybe::Maybe<ob::PathPtr>();
  }

  int num_samples = 100;
  int i = 0;

  double start_d_to_goal = goals.distance(start);

  double max_motion = space->getLongestValidSegmentLength() * 2;

  ob::State* test(si_->allocState());
  ob::State* best(si_->allocState());
  double best_ev = 0;
  bool found_ok = false;

  ob::StateSamplerPtr sampler_ = si_->allocStateSampler();
  vppc->setProbabilityThreshold(std::numeric_limits<double>::max());
  double best_cost = std::numeric_limits<double>::max();

  double goal_sampled = false;

  // while(i < num_samples)
  for (i = 0; i < num_samples; i++) {
    bool checking_goal_directly = (i == 0);
    bool near_goal = goals.distance(start) <= max_motion * 1.5;
    if (i == 0) {
      goal_sampled = true;
      si_->copyState(test, goals.get());
      goal_sampled = true;
    } else {
      sampler_->sampleUniform(test);

      // uniformly sample motion directions, not states
      double* testv = test->as<ob::RealVectorStateSpace::StateType>()->values;
      const double* startv = start->as<ob::RealVectorStateSpace::StateType>()->values;
      for (int i = 0; i < 7; i++) {
        testv[i] += startv[i];
      }
    }

    double motion_dist = si_->distance(start.get(), test);

    if (motion_dist > max_motion) {
      space->interpolate(start.get(), test, max_motion / motion_dist, test);
    }

    double progress_d = start_d_to_goal - goals.distance(test);

    if (progress_d <= 0 || progress_d <= best_ev) {
      if (i > 2 && !near_goal) {
        i--;
      }

      continue;
    }

    size_t num_interp_points = 2;
    std::vector<ob::State*> dpath;
    si_->getMotionStates(start.get(), test, dpath, num_interp_points, true, true);
    size_t col_index;
    double num_vox = vppc->getPathCost(dpath, col_index);
    si_->freeStates(dpath);

    if (num_vox > VOX_CONTROLLER_THRESHOLD) {
      continue;
    }

    // double ev = (1.0 - pcol) * (progress_d);
    // double ev = (1.0 - pcol);
    if (num_vox < best_cost) {
      si_->copyState(best, test);
      best_cost = num_vox;
      found_ok = true;
      // best_pcol = pcol;
    }
    if (checking_goal_directly) {
      break;
    }
  }

  si_->freeState(test);

  if (!found_ok) {
    return Maybe::Maybe<ob::PathPtr>();
  }

  bool debug = false;
  if (debug) {
    std::vector<ob::State*> dpath;
    dpath.push_back(start.get());
    dpath.push_back(best);

    size_t col_index;
    std::cout << "best cost " << best_cost << ", when reevaluated ";
    std::cout << vppc->getPathCost(dpath, col_index) << "\n";

    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    std::string unused;
    std::getline(std::cin, unused);
  }

  if (debug && goal_sampled) {
    std::vector<ob::State*> dpath;
    dpath.push_back(start.get());
    dpath.push_back(best);

    size_t col_index;
    std::cout << "best cost " << best_cost << ", when reevaluated ";
    std::cout << vppc->getPathCost(dpath, col_index) << "\n";
    std::cout << "distance from goal " << goals.distance(best) << "\n";

    victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    std::string unused;
    std::getline(std::cin, unused);
  }

  auto path(std::make_shared<og::PathGeometric>(si_));
  path->append(start.get());
  path->append(best);

  return Maybe::Maybe<ob::PathPtr>(path);
}

/************************************
 **        Victor LBKPICE          **
 ***********************************/

VictorLBKPiece::VictorLBKPiece(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) { initializePlanner(); }

void VictorLBKPiece::initializePlanner() {
  vv_ptr = std::make_shared<VictorConservativeValidator>(si_, victor_model_);
  setupSpaceInformation();

  planner_ = std::make_shared<og::LBKPIECE1>(si_);
  planner_->setup();
}

void VictorLBKPiece::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  planner_->setProblemDefinition(pdef_);
}

/***********************************************
 **               Victor LazyRRTF                **
 ***********************************************/

VictorLazyRRTF::VictorLazyRRTF(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) { initializePlanner(); }

void VictorLazyRRTF::initializePlanner() {
  // spi_ptr->setStateValidityChecker(v_ptr);
  // spi_ptr->setMotionValidator(v_ptr);
  std::shared_ptr<og::LazyRRTF> lrrtf = std::make_shared<og::LazyRRTF>(si_);
  pv_ = std::make_shared<VictorPathProbCol>(si_, victor_model_);
  lrrtf->setPathValidator(pv_);
  planner_ = lrrtf;
  planner_->setup();
  threshold = 1.0;

  vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
  si_->setStateValidityChecker(vv_ptr->getptr());
}

Maybe::Maybe<ob::PathPtr> VictorLazyRRTF::planPath(ompl::base::ScopedState<> start, Goals goals) {
  std::cout << "Using lazyRRTF planner\n";
  preparePlanner(start, goals);
  ob::PathPtr path;
  int planning_time = PLANNING_TIMEOUT;

  threshold = 1.0;
  pv_->setProbabilityThreshold(threshold);
  ob::PlannerStatus solved = planner_->solve(planning_time);

  while (solved) {
    threshold -= 0.1;
    if (threshold <= 0.0) {
      break;
    }

    std::cout << "threshold " << threshold << "\n";
    pv_->setProbabilityThreshold(threshold);
    solved = planner_->solve(planning_time);
  }

  if (threshold == 1.0) {
    std::cout << "No solution could be found" << std::endl;
    return Maybe::Maybe<ob::PathPtr>();
  }

  path = pdef_->getSolutionPath();

  return Maybe::Maybe<ob::PathPtr>(path);
}

void VictorLazyRRTF::preparePlanner(ob::ScopedState<> start, Goals goals) {
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);
  planner_->setProblemDefinition(pdef_);
}

/************************************
 **          Victor PRM            **
 ***********************************/

VictorPRM::VictorPRM(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) { initializePlanner(); }

void VictorPRM::initializePlanner() {
  vv_ptr = std::make_shared<VictorConservativeValidator>(si_, victor_model_);
  setupSpaceInformation();

  planner_ = std::make_shared<og::PRM>(si_);
  planner_->setup();
}

void VictorPRM::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  planner_->setProblemDefinition(pdef_);
}

/************************************
 **       Victor RRTConnect        **
 ***********************************/

VictorRRTConnect::VictorRRTConnect(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) { initializePlanner(); }

void VictorRRTConnect::initializePlanner() {
  vv_ptr = std::make_shared<VictorSampledWorldValidator>(si_, victor_model_);
  setupSpaceInformation();

  planner_ = std::make_shared<og::RRTConnect>(si_);
  planner_->setup();
}

void VictorRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  planner_->setProblemDefinition(pdef_);
}

/*************************************
 **         DiversePlanner         **
 ************************************/
DiversePlanner::DiversePlanner(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) { initializePlanner(); }

void DiversePlanner::initializePlanner() {
  vv_sampled_world_ptr = std::make_shared<VictorSampledWorldValidator>(si_, victor_model_);
  vv_ptr = vv_sampled_world_ptr;
  setupSpaceInformation();
  path_prob_validator = std::make_shared<VictorPathProbCol>(si_, victor_model_);

  planner_ = std::make_shared<og::RRTConnect>(si_);
  planner_->setup();
}

void DiversePlanner::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  planner_->setProblemDefinition(pdef_);
}

bool DiversePlanner::sampleBlockingWorld(std::vector<ompl::base::PathPtr> paths, double time_allowed) {
  arc_utilities::Stopwatch stopwatch;
  victor_model_->sampleValidWorld();
  int i = 0;
  while (isPathSetValid(paths)) {
    victor_model_->sampleValidWorld();
    i++;
    if (stopwatch() > time_allowed) {
      std::cout << "Could not sample a blocking world\n";
      return false;
    }
  }
  std::cout << "Sampled a blocking world after " << i << "\n";
  return true;
}

bool DiversePlanner::isPathSetValid(std::vector<ompl::base::PathPtr> paths) {
  for (auto& path : paths) {
    if (vv_sampled_world_ptr->isPathValid(path->as<og::PathGeometric>()->getStates())) {
      return true;
    }
  }
  return false;
}

Maybe::Maybe<ob::PathPtr> DiversePlanner::planSinglePathInSampledWorld(ob::ScopedState<> start, Goals goals) {
  preparePlanner(start, goals);
  ob::PlannerStatus solved = planner_->solve(10);
  if (!solved) {
    return Maybe::Maybe<ob::PathPtr>();
  }

  ob::PathPtr path = pdef_->getSolutionPath();
  simp_->shortcutPath(*(path->as<og::PathGeometric>()), 200, 200);
  return Maybe::Maybe<ob::PathPtr>(path);
}

Maybe::Maybe<ob::PathPtr> DiversePlanner::planPath(ompl::base::ScopedState<> start, Goals goals) {
  int num_paths = 10;

  std::cout << "Using Sampling planner\n";
  std::vector<ob::PathPtr> paths;
  std::vector<double> path_costs;

  arc_utilities::Stopwatch stopwatch;
  double planning_time = PLANNING_TIMEOUT;
  // double planning_time = 120;

  // for(int i=0; i < num_paths; i++)
  while (stopwatch() < planning_time && paths.size() < num_paths) {
    // victor_model_->sampleValidWorld();
    if (!sampleBlockingWorld(paths, stopwatch() - planning_time)) {
      break;
    }
    victor_model_->hidePath();
    victor_model_->gvl->visualizeMap(SAMPLED_WORLD_MAP);

    // auto maybe_path = VictorPlanner::planPath(start, goals);
    auto maybe_path = planSinglePathInSampledWorld(start, goals);

    if (maybe_path.Valid()) {
      paths.push_back(maybe_path.Get());
      size_t col_index;
      path_costs.push_back(
          path_prob_validator->getPathCost(maybe_path.Get()->as<og::PathGeometric>()->getStates(), col_index));

      victor_model_->visPath(omplPathToDoublePath(maybe_path.Get()->as<og::PathGeometric>(), si_));

      std::string unused;
      std::cout << "Waiting for user input to start...\n";
      std::getline(std::cin, unused);
    } else {
      std::cout << "No path found\n";
    }
  }

  victor_model_->hidePath();

  size_t best_ind = std::distance(path_costs.begin(), std::min_element(path_costs.begin(), path_costs.end()));
  std::cout << "Path costs: \n";
  for (auto cost : path_costs) {
    std::cout << cost << "\n";
  }
  std::cout << "best cost is in ind: " << best_ind << "\n";

  if (paths.size() > 0) {
    return Maybe::Maybe<ob::PathPtr>(paths[best_ind]);
  }
  std::cout << "Path set is empty\n";
  return Maybe::Maybe<ob::PathPtr>();
}

/*************************************
 **  Victor SamplingRRTConnect      **
 ************************************/
VictorSamplingRRTConnect::VictorSamplingRRTConnect(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) {
  initializePlanner();
}

void VictorSamplingRRTConnect::initializePlanner() {
  vv_ptr = std::make_shared<VictorSampledWorldValidator>(si_, victor_model_);
  setupSpaceInformation();
  path_prob_validator = std::make_shared<VictorPathProbCol>(si_, victor_model_);

  planner_ = std::make_shared<og::RRTConnect>(si_);
  planner_->setup();
}

void VictorSamplingRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  planner_->setProblemDefinition(pdef_);
}

Maybe::Maybe<ob::PathPtr> VictorSamplingRRTConnect::planPath(ompl::base::ScopedState<> start, Goals goals) {
  int num_paths = 10;

  std::cout << "Using Sampling planner\n";
  std::vector<ob::PathPtr> paths;
  std::vector<double> path_costs;

  for (int i = 0; i < num_paths; i++) {
    victor_model_->sampleValidWorld();
    auto maybe_path = VictorPlanner::planPath(start, goals);
    if (maybe_path.Valid()) {
      paths.push_back(maybe_path.Get());
      size_t col_index;
      path_costs.push_back(
          path_prob_validator->getPathCost(maybe_path.Get()->as<og::PathGeometric>()->getStates(), col_index));

      victor_model_->visPath(omplPathToDoublePath(maybe_path.Get()->as<og::PathGeometric>(), si_));

      // std::string unused;
      // std::cout << "Waiting for user input to start...\n";
      // std::getline(std::cin, unused);
    }
  }

  victor_model_->hidePath();

  size_t best_ind = std::distance(path_costs.begin(), std::min_element(path_costs.begin(), path_costs.end()));
  std::cout << "Path costs: \n";
  for (auto cost : path_costs) {
    std::cout << cost << "\n";
  }
  std::cout << "best cost is in ind: " << best_ind << "\n";

  if (paths.size() > 0) {
    return Maybe::Maybe<ob::PathPtr>(paths[best_ind]);
  }
  return Maybe::Maybe<ob::PathPtr>();
}

/***************************************
 **    Victor ThresholdRRTConnect     **
 ***************************************/

VictorThresholdRRTConnect::VictorThresholdRRTConnect(GpuVoxelsVictor* victor_model, bool use_vox, bool use_prob_col)
    : VictorPlanner(victor_model) {
  use_vox_ = use_vox;
  use_prob_col_ = use_prob_col;
  initializePlanner();
}

void VictorThresholdRRTConnect::initializePlanner() {
  auto vstv = std::make_shared<VictorStateThresholdValidator>(si_, victor_model_);
  vstv->use_vox = use_vox_;
  vstv->use_prob_col = use_prob_col_;
  vv_ptr = vstv;
  setupSpaceInformation();

  auto crrt(std::make_shared<og::cRRTConnect>(si_));
  crrt->setRange(0.1);
  planner_ = crrt;
  planner_->setup();
  if (use_vox_) {
    vppc = std::make_shared<VictorPathVox>(si_, victor_model_);
  } else if (use_prob_col_) {
    vppc = std::make_shared<VictorPathProbCol>(si_, victor_model_);
  }
}

Maybe::Maybe<ob::PathPtr> VictorThresholdRRTConnect::planPath(ompl::base::ScopedState<> start, Goals goals) {
  std::cout << "Using threshold planner\n";
  preparePlanner(start, goals);

  ob::PathPtr path;
  double planning_time = PLANNING_TIMEOUT;
  double eps = 0.0001;

  double threshold = std::numeric_limits<double>::max();

  ob::PlannerStatus solved(true, false);
  bool anySolution = false;
  VictorStateThresholdValidator* vv_thresh = dynamic_cast<VictorStateThresholdValidator*>(vv_ptr.get());
  vv_thresh->setCostThreshold(threshold);

  arc_utilities::Stopwatch stopwatch;
  double time_left;

  std::string profile_optim_name = name + " optimal value";
  PROFILE_START(profile_optim_name);
  PROFILE_RECORD_DOUBLE(profile_optim_name, -1);

  while ((time_left = (planning_time - stopwatch())) > 0) {
    // threshold -= 0.01;
    // if(threshold <= 0.0){
    //     std::cout << "0 cost path found. Exiting early\n";
    //     break;
    // }

    std::cout << "threshold " << threshold << "\n";

    solved = planner_->solve(time_left);

    if (solved) {
      anySolution = true;
      path = pdef_->getSolutionPath();
      double tmp_threshold = vv_thresh->getPathCost(path->as<og::PathGeometric>()) - eps;
      // std::cout << "Max cost on path: " << tmp_threshold << "\n";
      if (tmp_threshold < threshold) {
        threshold = tmp_threshold;
      }
      PROFILE_RECORD_DOUBLE(profile_optim_name, threshold + eps);
      // std::cout << "Max cost on path: " << threshold + eps << "\n";
      vv_thresh->setCostThreshold(threshold);
      preparePlanner(start, goals);
      if (!vv_thresh->isValid(start.get()) || !vv_thresh->isValid(goals.get())) {
        std::cout << "Found best path for given start/goal with threshold " << (threshold + eps) << "\n";
        vv_thresh->setCostThreshold(threshold + 2 * eps);
        break;
      }
    }
  }
  std::cout << "Planning finished\n";

  if (!anySolution) {
    std::cout << "No solution could be found" << std::endl;

    return Maybe::Maybe<ob::PathPtr>();
  }

  (path->as<og::PathGeometric>())->interpolate();
  std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states before smoothing\n";
  simp_->shortcutPath(*(path->as<og::PathGeometric>()), 100, 30);
  // std::cout << "done simplifying\n";
  std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states after smoothing\n";

  return Maybe::Maybe<ob::PathPtr>(path);
}

void VictorThresholdRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);
  planner_->setProblemDefinition(pdef_);
}

Maybe::Maybe<ob::PathPtr> VictorThresholdRRTConnect::localControl(ompl::base::ScopedState<> start, Goals goals) {
  if (use_prob_col_) {
    return VictorPlanner::localControl(start, goals);
  } else if (use_vox_) {
    return voxLocalControl(start, goals);
  } else {
    assert(false);
  }
}

/***************************************
 **    Victor MotionCostRRTConnect    **
 ***************************************/

VictorMotionCostRRTConnect::VictorMotionCostRRTConnect(GpuVoxelsVictor* victor_model) : VictorPlanner(victor_model) {}

void VictorMotionCostRRTConnect::smooth(ob::PathPtr& path) {
  // rplanner_->pv_->do_delay = true;

  // rplanner_->pv_->setProbabilityThreshold(std::numeric_limits<double>::max());
  // rplanner_->pv_->setProbabilityThreshold(unsmoothed_path_prob);

  // double unsmoothed_path_prob = rplanner_->pv_->getPathCost(path->as<og::PathGeometric>()->getStates(), col_index);
  // double unsmoothed_path_prob = best_cost;
  // size_t col_index;
  // Note, this could be higher due to densification.

  std::cout << "pv thresh after planning: " << rplanner_->pv_->threshold << "\n";

  bool tmp = rplanner_->pv_->do_delay;
  rplanner_->pv_->do_delay = false;

  og::PathSmoother cost_simp(si_, rplanner_->pv_.get());
  cost_simp.shortcutPath(*(path->as<og::PathGeometric>()), SMOOTHING_ITERATIONS);

  rplanner_->pv_->do_delay = tmp;

  // double smoothed_path_prob = rplanner_->pv_->getPathCost(path->as<og::PathGeometric>()->getStates(), col_index);

  // std::cout << "Path has " << path->as<og::PathGeometric>()->getStates().size() << " states after smoothing with cost
  // " << smoothed_path_prob << "\n";

  // assert(smoothed_path_prob <= unsmoothed_path_prob);

  rplanner_->pv_->do_delay = false;
}

Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planAnytime(ob::ScopedState<> start, Goals goals) {
  preparePlanner(start, goals);
  ob::PathPtr path;
  double planning_time = PLANNING_TIMEOUT;
  double eps = 0.0001;
  double threshold = std::numeric_limits<double>::max();

  bool any_solution = false;

  rplanner_->setProbabilityThreshold(threshold);
  double best_cost = threshold;

  arc_utilities::Stopwatch stopwatch;
  double time_left;

  std::string profile_optim_name = name + " optimal value";
  PROFILE_START(profile_optim_name);
  PROFILE_RECORD_DOUBLE(profile_optim_name, -1);

  while ((time_left = (planning_time - stopwatch())) > 0) {
    std::cout << "threshold " << threshold << "\n";

    ob::PlannerStatus solved = planner_->solve(time_left);

    if (solved) {
      any_solution = true;

      ob::PathPtr ptmp = pdef_->getSolutionPath();
      double path_cost = rplanner_->path_cost;

      std::cout << "Path cost: " << path_cost << "\n";
      if (path_cost < threshold) {
        PROFILE_RECORD_DOUBLE(profile_optim_name, path_cost);
        best_cost = path_cost;
        threshold = (path_cost - eps);
        path = ptmp;
        rplanner_->setProbabilityThreshold(threshold);
        // std::cout << "pv thresh: " << rplanner_->pv_->threshold << "\n";
      } else {
        std::cout << "This should not happen with planner modifications\n";
        assert(false);
      }

      if (threshold <= 0.0) {
        std::cout << "Early termination because 0 cost path found!\n";
        break;
      }

      preparePlanner(start, goals);
    }
  }
  std::cout << "Planning finished\n";

  if (!any_solution) {
    std::cout << "No solution could be found" << std::endl;

    return Maybe::Maybe<ob::PathPtr>();
  }
  rplanner_->pv_->setProbabilityThreshold(best_cost);
  (path->as<og::PathGeometric>())->interpolate();
  return Maybe::Maybe<ob::PathPtr>(path);
}

Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planUp(ob::ScopedState<> start, Goals goals) {
  std::cout << "Planning up!\n";
  preparePlanner(start, goals);
  ob::PathPtr path;
  double planning_time = PLANNING_TIMEOUT;
  double eps = 0.0001;
  double initial_thresh = 0.3;
  double threshold = initial_thresh;

  bool any_solution = false;

  rplanner_->setProbabilityThreshold(threshold);
  double best_cost = threshold;

  arc_utilities::Stopwatch stopwatch;
  double time_left;

  std::string profile_optim_name = name + " optimal value";
  PROFILE_START(profile_optim_name);
  // PROFILE_RECORD_DOUBLE(profile_optim_name, -1);

  while ((time_left = (planning_time - stopwatch())) > 0) {
    std::cout << "threshold " << threshold << "\n";

    double iter_plan_time = any_solution ? time_left : time_left / 4;

    ob::PlannerStatus solved = planner_->solve(iter_plan_time);

    if (solved) {
      any_solution = true;

      ob::PathPtr ptmp = pdef_->getSolutionPath();
      double path_cost = rplanner_->path_cost;

      std::cout << "Path found with cost: " << path_cost << "\n";
      if (path_cost < threshold) {
        PROFILE_RECORD_DOUBLE(profile_optim_name, path_cost);
        best_cost = path_cost;
        threshold = (path_cost - eps);
        path = ptmp;
        rplanner_->setProbabilityThreshold(threshold);
        // std::cout << "pv thresh: " << rplanner_->pv_->threshold << "\n";
      } else {
        std::cout << "Shouldn't happend with this rrtconnect\n";
        assert(false);
      }

      if (threshold <= 0.0) {
        std::cout << "Early termination because 0 cost path found!\n";
        break;
      }

      preparePlanner(start, goals);
    } else if (!any_solution) {
      threshold = (stopwatch() / planning_time) * cost_upper_bound + initial_thresh;
      rplanner_->setProbabilityThreshold(threshold);
      std::cout << "No solution found, raising threshold to " << threshold << "\n";
    }
  }
  std::cout << "Planning finished\n";

  if (!any_solution) {
    std::cout << "No solution could be found" << std::endl;

    return Maybe::Maybe<ob::PathPtr>();
  }
  rplanner_->pv_->setProbabilityThreshold(best_cost);
  (path->as<og::PathGeometric>())->interpolate();
  return Maybe::Maybe<ob::PathPtr>(path);
}

Maybe::Maybe<ob::PathPtr> VictorMotionCostRRTConnect::planPath(ompl::base::ScopedState<> start, Goals goals) {
  Maybe::Maybe<ob::PathPtr> path;
  if (use_anytime_planner)
    path = planAnytime(start, goals);
  else
    path = planUp(start, goals);

  if (!path.Valid()) {
    return path;
  }

  std::cout << "Path has " << path.Get()->as<og::PathGeometric>()->getStates().size()
            << " states before smoothing with cost " << rplanner_->path_cost << ",... ";

  smooth(path.Get());
  std::cout << " and " << path.Get()->as<og::PathGeometric>()->getStates().size() << " states after smoothing\n";

  return Maybe::Maybe<ob::PathPtr>(path);
}

void VictorMotionCostRRTConnect::preparePlanner(ob::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);
  planner_->setProblemDefinition(pdef_);
}

/***************************************
 **    Victor VoxCostRRTConnect    **
 ***************************************/
VictorVoxCostRRTConnect::VictorVoxCostRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorMotionCostRRTConnect(victor_model) {
  std::cout << "Using Voxel Cost planner\n";
  initializePlanner();
  cost_upper_bound = 400;
}

void VictorVoxCostRRTConnect::initializePlanner() {
  vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
  setupSpaceInformation();

  std::shared_ptr<og::CostRRTConnect> mcrrt = std::make_shared<og::CostRRTConnect>(si_);
  vppc = std::make_shared<VictorPathVox>(si_, victor_model_);
  std::shared_ptr<VictorPathVox> vppc_vox = std::make_shared<VictorPathVox>(si_, victor_model_);
  mcrrt->setPathValidator(vppc_vox);
  planner_ = mcrrt;
  rplanner_ = planner_->as<og::CostRRTConnect>();
  planner_->setup();
  // std::cout << "Planner max extend dist: " << mcrrt->getRange() << "\n";
}

Maybe::Maybe<ob::PathPtr> VictorVoxCostRRTConnect::localControl(ob::ScopedState<> start, Goals goals) {
  return voxLocalControl(start, goals);
}

/****************************************
 **    Victor ProbColCostRRTConnect    **
 ****************************************/
VictorProbColCostRRTConnect::VictorProbColCostRRTConnect(GpuVoxelsVictor* victor_model)
    : VictorMotionCostRRTConnect(victor_model) {
  std::cout << "Using Probabiltiy of Collision planner\n";
  initializePlanner();
  cost_upper_bound = 1.0;
}

void VictorProbColCostRRTConnect::initializePlanner() {
  vv_ptr = std::make_shared<VictorValidator>(si_, victor_model_);
  setupSpaceInformation();

  std::shared_ptr<og::CostRRTConnect> mcrrt = std::make_shared<og::ProbColRRTConnect>(si_);
  vppc = std::make_shared<VictorPathProbCol>(si_, victor_model_);

  mcrrt->setPathValidator(vppc);
  planner_ = mcrrt;
  rplanner_ = planner_->as<og::CostRRTConnect>();
  planner_->setup();
  // std::cout << "Planner max extend dist: " << mcrrt->getRange() << "\n";
}

/************************************
 **          Victor RRTStar        **
 ***********************************/
VictorRRTStar::VictorRRTStar(GpuVoxelsVictor* victor_model, bool use_prob_cost) : VictorPlanner(victor_model) {
  use_prob_cost_ = use_prob_cost;
  initializePlanner();
}

void VictorRRTStar::initializePlanner() {
  auto rrtstar_planner = std::make_shared<og::cRRTstar>(si_);
  if (use_prob_cost_) {
    std::cout << "RRT star with prob col\n";
    // std::shared_ptr<VictorPathProbCol> pv;
    vppc = std::make_shared<VictorPathProbCol>(si_, victor_model_);
    vppc->setProbabilityThreshold(1.0);
    opt = std::make_shared<VictorObjective>(si_, vppc);
    opt->is_prob_cost = true;
  } else {
    std::cout << "vox cost not yet implemented. No controller yet\n";
  }

  planner_ = rrtstar_planner;
  planner_->setup();
}

void VictorRRTStar::preparePlanner(ompl::base::ScopedState<> start, Goals goals) {
  planner_->clear();
  pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
  pdef_->setStartAndGoalStates(start, goals);

  pdef_->setOptimizationObjective(opt);
  planner_->setProblemDefinition(pdef_);
}

Maybe::Maybe<ob::PathPtr> VictorRRTStar::planPath(ompl::base::ScopedState<> start, Goals goals) {
  preparePlanner(start, goals);
  ob::PlannerStatus solved = planner_->solve(PLANNING_TIMEOUT);
  if (solved) {
    return Maybe::Maybe<ob::PathPtr>(pdef_->getSolutionPath());
  }
  return Maybe::Maybe<ob::PathPtr>();
}
