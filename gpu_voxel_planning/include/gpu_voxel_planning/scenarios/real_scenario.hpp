#ifndef GVP_REAL_SCENARIOS_HPP
#define GVP_REAL_SCENARIOS_HPP

#include "gpu_voxel_planning/scenarios/scenarios.hpp"

namespace GVP {

class RealScenario : public Scenario {
 public:
  RealState s;
  ObstacleConfiguration known_obstacles;
  ObstacleConfiguration unknown_obstacles;

  std::string belief_name;

  explicit RealScenario(const std::string &config_file);

  void initFakeVictor(RosInterface& ri);
  void setPrior(ObstacleConfiguration& unknown_obstacles, BeliefParams bp);
  // virtual void setPrior(BeliefParams bp);

  virtual void validate();

  virtual void viz(const GpuVoxelRvizVisualizer& viz) override;

  virtual State& getState() override { return s; }

  virtual const State& getState() const override { return s; }

  virtual RealState& getRealState() { return s; }

  virtual const RealState& getRealState() const { return s; }

  void addLeftArm();

  DenseGrid loadPointCloudFromFile();
};

/****************************************
 **         Table With Box
 ****************************************/
class RealTable : public RealScenario {
 public:
  const std::string name;

  RealTable(BeliefParams bp);

  virtual std::string getName() const override { return name; }

  Object getTable();
};

/****************************************
 **         Emtpy
 ****************************************/
class RealEmpty : public RealScenario {
 public:
  const std::string name;

  explicit RealEmpty(BeliefParams bp);

  virtual std::string getName() const override { return name; }

  Object getTable();
};

/****************************************
**      Real ShapeRequest Scenario
****************************************/
class RealShapeRequestScenario : public RealScenario {
  const std::string name;

 public:
  explicit RealShapeRequestScenario(const BeliefParams& bp);
  std::string getName() const override { return "ShapeRequest"; }
  static Object getObstacles(const std::string& topic);
  std::vector<robot::JointValueMap> getPossibleGoals() const override;

  bool completed() const override;
};
}  // namespace GVP

#endif

