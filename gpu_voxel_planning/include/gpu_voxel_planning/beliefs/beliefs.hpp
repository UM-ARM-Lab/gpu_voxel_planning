#ifndef GVP_BELIEFS_HPP
#define GVP_BELIEFS_HPP

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <random>
#include <utility>

#include "gpu_voxel_planning/obstacles/obstacles.hpp"
#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "gpu_voxel_planning/scenarios/goal.hpp"

namespace GVP {
enum BeliefType { CHS, IID, Obstacle, Bonkers, MoEObstacle, MoEBonkers, Deterministic, ShapeCompletion };

struct BeliefParams {
  BeliefType belief_type;
  std::vector<double> bias;
  double noise;

  BeliefParams() : belief_type(BeliefType::CHS), bias(std::vector<double>{0, 0, 0}), noise(0){};

  explicit BeliefParams(BeliefType bt, std::vector<double> bias = std::vector<double>{0, 0, 0}, double noise = 0)
      : belief_type(bt), bias(std::move(bias)), noise(noise) {}

  [[nodiscard]] std::string toString() const {
    std::map<BeliefType, std::string> m{{BeliefType::CHS, "CHS"},
                                        {BeliefType::IID, "IID"},
                                        {BeliefType::Obstacle, "Obstacle"},
                                        {BeliefType::Bonkers, "Bonkers"},
                                        {BeliefType::MoEObstacle, "MoE"},
                                        {BeliefType::MoEBonkers, "MoEBonkers"},
                                        {BeliefType::Deterministic, "Deterministic"},
                                        {BeliefType::ShapeCompletion, "ShapeCompletion"}};

    std::string name = m[belief_type] + "_" + std::to_string(bias[0]) + "_" + std::to_string(bias[1]) + "_" +
                       std::to_string(bias[2]) + "_" + std::to_string(noise);
    return name;
  }
};

class Belief {
 public:
  virtual void viz(const GpuVoxelRvizVisualizer &viz) = 0;

  virtual double calcProbFree(const DenseGrid &volume) = 0;

  virtual void updateFreeSpace(const DenseGrid &new_free) = 0;

  virtual void updateCollisionSpace(Robot &robot, size_t first_link_in_collision) = 0;

  [[nodiscard]] virtual DenseGrid sampleState() const = 0;

  // virtual std::string getName() const = 0;
  virtual ~Belief() = default;

  [[nodiscard]] virtual std::unique_ptr<Belief> clone() const = 0;

  [[nodiscard]] virtual std::vector<robot::JointValueMap> getPossibleGoals() const {
    throw std::logic_error("getPossibleGoals is not implemented for this belief type\n");
  }

  virtual void syncBelief() {}
};

class EmptyBelief : public Belief {
 public:
 public:
  EmptyBelief() = default;

  void viz(const GpuVoxelRvizVisualizer &viz) override {}

  double calcProbFree(const DenseGrid &volume) override { return 1.0; }

  void updateFreeSpace(const DenseGrid &new_free) override {}

  void updateCollisionSpace(Robot &robot, const size_t first_link_in_collision) override {}

  [[nodiscard]] DenseGrid sampleState() const override { return DenseGrid(); }

  // virtual std::string getName() const = 0;
  [[nodiscard]] std::unique_ptr<Belief> clone() const override { return std::make_unique<EmptyBelief>(); }
};

class ObstacleBelief : public Belief {
 public:
  std::vector<ObstacleConfiguration> particles;
  std::vector<double> weights;

 public:
  ObstacleBelief() = default;

  ObstacleBelief(const ObstacleConfiguration &oc, double noise, const std::vector<double> &bias);

  [[nodiscard]] std::unique_ptr<Belief> clone() const override { return cloneObstacleBelief(); }

  [[nodiscard]] std::unique_ptr<ObstacleBelief> cloneObstacleBelief() const;

  void addElem(const ObstacleConfiguration &obs, double weight);

  double calcProbFree(const DenseGrid &volume) override;

  void updateFreeSpace(const DenseGrid &new_free) override;

  [[nodiscard]] std::vector<std::vector<double>> getParticleVectors() const;

  /**
   * Computes weights of the current posterior particles given a prior
   * Note!! This assumes the prior is weighted evently, which is not the case
   **/
  [[nodiscard]] std::vector<double> kernelDensityEstimate(std::vector<std::vector<double>> prior,
                                                          double bandwidth) const;

  void updateCollisionSpace(Robot &robot, size_t first_link_in_collision) override;

  [[nodiscard]] DenseGrid sampleState() const override;

  void viz(const GpuVoxelRvizVisualizer &viz) override;

  // protected:
  [[nodiscard]] std::vector<double> cumSum() const;
};

class ShapeCompletionBelief : public Belief {
 public:
  DenseGrid known_free;
  std::vector<DenseGrid> chss;
  std::vector<DenseGrid> sampled_particles;
  ros::Publisher free_space_publisher;
  int num_samples = 10;
  std::optional<std::vector<robot::JointValueMap>> goal_configs;
  std::vector<TSR> goal_tsrs;

 public:
  ShapeCompletionBelief();

  double calcProbFree(const DenseGrid &volume) override;

  void updateFreeSpace(const DenseGrid &new_free) override;

  void updateCollisionSpace(Robot &robot, size_t first_link_in_collision) override;

  DenseGrid sampleState() const override;

  void viz(const GpuVoxelRvizVisualizer &viz) override;

  std::unique_ptr<Belief> clone() const override;

  void requestCompletions();

  [[nodiscard]] std::vector<robot::JointValueMap> getPossibleGoals() const override;

  void syncBelief() override;
};

/*********************************
 **         IID Belief          **
 ********************************/
class IIDBelief : public ObstacleBelief {
 public:
  IIDBelief(const ObstacleConfiguration &oc, const double noise, const std::vector<double> &bias)
      : ObstacleBelief(oc, noise, bias) {}

  void updateFreeSpace(const DenseGrid &new_free) override {}

  void updateCollisionSpace(Robot &robot, const size_t first_link_in_collision) override {}

  [[nodiscard]] DenseGrid sampleState() const override { return DenseGrid(); }
};

/*********************************
 **         CHS Belief          **
 ********************************/

class ChsBelief : public Belief {
 public:
  std::vector<DenseGrid> chs;
  DenseGrid known_free;

 public:
  std::unique_ptr<ChsBelief> cloneChsBelief() const;

  std::unique_ptr<Belief> clone() const override { return cloneChsBelief(); }

  void viz(const GpuVoxelRvizVisualizer &viz) override;

  double calcProbFree(const DenseGrid &volume) override;

  void updateFreeSpace(const DenseGrid &new_free) override;

  void updateCollisionSpace(Robot &robot, size_t first_link_in_collision) override;

  virtual void addChs(Robot &robot, size_t first_link_in_collision);

  DenseGrid sampleState() const override;
};

/*********************************
 **  Mixture Of Experts Belief  **
 ********************************/
class MoEBelief : public Belief {
 public:
  std::vector<double> weights;
  ChsBelief expert_chs;
  ObstacleBelief expert_particle;
  std::vector<Belief *> experts;
  std::vector<std::vector<double>> particle_prior;

  // public:
  //     MoEBelief(){}

 public:
  MoEBelief(const ObstacleConfiguration &oc, double noise, const std::vector<double> &bias);

  MoEBelief(ChsBelief chsb, ObstacleBelief obsb) : expert_chs(std::move(chsb)), expert_particle(std::move(obsb)) {}

  std::unique_ptr<Belief> clone() const override;

  static double kernelDensityLikelihood(const std::vector<std::vector<double>> &prior,
                                        const std::vector<std::vector<double>> &posterior,
                                        std::vector<double> posterior_weights, double bandwidth);

  /* Returns a vector of the cumulative sum of the expert weights*/
  std::vector<double> cumSum() const;

  void viz(const GpuVoxelRvizVisualizer &viz) override;

  void updateWeights();

  double calcProbFree(const DenseGrid &volume) override;

  void updateFreeSpace(const DenseGrid &new_free) override;

  void updateCollisionSpace(Robot &robot, size_t first_link_in_collision) override;

  DenseGrid sampleState() const override;
};
}  // namespace GVP

#endif
