#ifndef GVP_SIMULATION_SCENARIOS_HPP
#define GVP_SIMULATION_SCENARIOS_HPP

#include "gpu_voxel_planning/scenarios/scenarios.hpp"

namespace GVP {

class SimulationScenario : public Scenario {
 public:
  SimulationState s;
  ObstacleConfiguration true_obstacles;
  ObstacleConfiguration known_obstacles;
  ObstacleConfiguration unknown_obstacles;

  std::string belief_name;

  SimulationScenario();

  static void initFakeVictor(RosInterface& ri);
  void setPrior(ObstacleConfiguration& unknown_obstacles_prior, const BeliefParams& bp);
  virtual void validate();
  void viz(const GpuVoxelRvizVisualizer& viz) override;
  virtual DenseGrid& getTrueObstacles() { return true_obstacles.occupied; }
  virtual const DenseGrid& getTrueObstacles() const { return true_obstacles.occupied; }
  virtual SimulationState& getSimulationState() { return s; }
  virtual const SimulationState& getSimulationState() const { return s; }
  State& getState() override { return s; }
  const State& getState() const override { return s; }
  void addLeftArm();

 protected:
  void combineObstacles();
};

/****************************************
 **         Empty
 ****************************************/
class Empty : public SimulationScenario {
  const std::string name;

 public:
  explicit Empty(const BeliefParams& bp);

  std::string getName() const override { return "Empty"; }
};

/****************************************
 **         Table With Box
 ****************************************/
class TableWithBox : public SimulationScenario {
 public:
  Vector3f cavecorner;
  Vector3f caveheight;
  Vector3f cavetopd;
  Vector3f cavesidedim;
  Vector3f cavesideoffset;
  const std::string name;

  explicit TableWithBox(const BeliefParams& bp, bool table_known = true, bool visible_cave_known = true,
                        bool full_cave_known = false);

  std::string getName() const override { return name; }

  static Object getTable();
  void setCaveDims();
  Object getVisibleCave();
  Object getCaveBack();
};

/****************************************
 **         SlottedWall
 ****************************************/
class SlottedWall : public SimulationScenario {
  const std::string name;

 public:
  explicit SlottedWall(const BeliefParams& bp);

  std::string getName() const override { return "SlottedWall"; }
  static Object getFrontWall();
  static Object getSlottedWall();
};

/****************************************
 **         Bookshelf
 ****************************************/
class Bookshelf : public SimulationScenario {
  const std::string name;

 public:
  explicit Bookshelf(const BeliefParams& bp);
  std::string getName() const override { return "Bookshelf"; }
  static Object getBookshelf();
  static Object getTable();
};

/****************************************
 **      Close Wall
 ****************************************/
class CloseWall : public SimulationScenario {
  const std::string name;

 public:
  explicit CloseWall(const BeliefParams& bp);
  std::string getName() const override { return "CloseWall"; }
  static Object getCloseWall();
};

/****************************************
**      ShapeRequest Scenario
****************************************/
class ShapeRequestScenario : public SimulationScenario {
  const std::string name;

 public:
  explicit ShapeRequestScenario(const BeliefParams& bp);
  std::string getName() const override { return "ShapeRequest"; }
  static Object getObstacles();
};

}  // namespace GVP

#endif
