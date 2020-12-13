#ifndef GVP_BELIEFS_HPP
#define GVP_BELIEFS_HPP

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <random>
#include <utility>

#include "gpu_voxel_planning/obstacles/obstacles.hpp"
#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"

namespace GVP {
enum BeliefType { CHS, IID, Obstacle, Bonkers, MoEObstacle, MoEBonkers, Deterministic };

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
                                        {BeliefType::Deterministic, "Deterministic"}};

    std::string name = m[belief_type] + "_" + std::to_string(bias[0]) + "_" + std::to_string(bias[1]) + "_" +
                       std::to_string(bias[2]) + "_" + std::to_string(noise);
    return name;
  }
};

class Belief {
 public:
  virtual void viz(const GpuVoxelRvizVisualizer& viz) = 0;

  virtual double calcProbFree(const DenseGrid& volume) = 0;

  virtual void updateFreeSpace(const DenseGrid& new_free) = 0;

  virtual void updateCollisionSpace(Robot& robot, size_t first_link_in_collision) = 0;

  [[nodiscard]] virtual DenseGrid sampleState() const = 0;

  // virtual std::string getName() const = 0;

  virtual ~Belief() = default;

  [[nodiscard]] virtual std::unique_ptr<Belief> clone() const = 0;
};

class EmptyBelief : public Belief {
 public:
 public:
  EmptyBelief() = default;
  void viz(const GpuVoxelRvizVisualizer& viz) override {}

  double calcProbFree(const DenseGrid& volume) override { return 1.0; }

  void updateFreeSpace(const DenseGrid& new_free) override {}

  void updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) override {}

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

  ObstacleBelief(const ObstacleConfiguration& oc, const double noise, const std::vector<double>& bias) {
    int num_samples = 100;  // HARDCODED PARAM
    std::mt19937 rng;
    std::normal_distribution<float> offset(0, noise);
    for (int i = 0; i < num_samples; i++) {
      ObstacleConfiguration sample = oc;
      for (auto& object : sample.obstacles) {
        object.shift(Vector3f(offset(rng) + bias[0], offset(rng) + bias[1], offset(rng) + bias[2]));
      }
      sample.remakeGrid();
      addElem(sample, 1.0);
    }
  }

  [[nodiscard]] std::unique_ptr<Belief> clone() const override { return cloneObstacleBelief(); }

  [[nodiscard]] std::unique_ptr<ObstacleBelief> cloneObstacleBelief() const {
    std::cout << "cloning obstacle belief\n";
    std::unique_ptr<ObstacleBelief> b = std::make_unique<ObstacleBelief>();
    b->particles = particles;
    b->weights = weights;
    return b;
  }

  void addElem(const ObstacleConfiguration& obs, double weight) {
    particles.push_back(obs);
    weights.push_back(weight);
  }

  double calcProbFree(const DenseGrid& volume) override {
    double agreement = 0;
    if (cumSum().back() <= 0) {
      return 1.0;
    }

    for (int i = 0; i < particles.size(); i++) {
      if (!particles[i].occupied.overlapsWith(&volume)) {
        agreement += weights[i];
      }
    }
    return agreement / cumSum().back();
  }

  void updateFreeSpace(const DenseGrid& new_free) override {
    PROFILE_START("Obstacle_belief_update_free");
    for (int i = 0; i < particles.size(); i++) {
      if (weights[i] == 0) {
        continue;
      }
      if (particles[i].occupied.collideWith(&new_free) > 0)  // heuristic for near collisions
      {
        weights[i] = 0;
      }
    }
    PROFILE_RECORD("Obstacle_belief_update_free");
  }

  [[nodiscard]] std::vector<std::vector<double>> getParticleVectors() const {
    std::vector<std::vector<double>> pvs;
    for (const auto& particle : particles) {
      pvs.push_back(particle.asParticle());
    }
    return pvs;
  }

  /**
   * Computes weights of the current posterior particles given a prior
   * Note!! This assumes the prior is weighted evently, which is not the case
   **/
  [[nodiscard]] std::vector<double> kernelDensityEstimate(std::vector<std::vector<double>> prior, double bandwidth) const {
    double normalizing = cumSum().back() * weights.size();
    if (normalizing <= 0) {
      return std::vector<double>(particles.size(), 0);
    }

    std::vector<double> new_weights;
    for (const auto& particle : particles) {
      double w_i = 0;
      for (int i = 0; i < weights.size(); i++) {
        // double d = EigenHelpers::Distance(particle.asParticle(), prior[i]);
        double density = 1;
        auto pv = particle.asParticle();
        for (int j = 0; j < pv.size(); j++) {
          double d = pv[j] - prior[i][j];
          density *= arc_helpers::EvaluateGaussianPDF(0, bandwidth, d);
        }
        w_i += density * weights[i];
      }
      new_weights.push_back(w_i / normalizing);
    }
    return new_weights;
  }

  void updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) override {
    PROFILE_START("Obstacle_belief_update_collision");
    auto prior = getParticleVectors();

    DistanceGrid dg;
    dg.mergeOccupied(&robot.occupied_space);
    for (auto& particle : particles) {
      particle.project(dg);
    }
    weights = kernelDensityEstimate(prior, 0.05);
    // recomputeWeights();
    std::cout << "Sum of particle weights is " << cumSum().back() << "\n";

    PROFILE_RECORD("Obstacle_belief_update_collision");
  }

  [[nodiscard]] DenseGrid sampleState() const override {
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_real_distribution<double> dist(0.0, cumSum().back());
    double r = dist(rng);

    auto cum_sum = cumSum();

    if (cum_sum.back() == 0) {
      return DenseGrid();
    }

    for (int i = 0; i < particles.size(); i++) {
      if (r <= cum_sum[i]) {
        return particles[i].occupied;
      }
    }
    throw std::logic_error("Particle sample not found in cum_sum");
  }

  void viz(const GpuVoxelRvizVisualizer& viz) override {
    for (int i = 0; i < particles.size(); i++) {
      double alpha = std::max(weights[i] / cumSum().back(), 1.0 / 10);
      if (weights[i] == 0) {
        alpha = 0;
      }
      std::string name = "belief_particle_" + std::to_string(i);
      viz.vizGrid(particles[i].occupied, name, makeColor(1.0, 0, 0, alpha));
    }
  }

  // protected:
  [[nodiscard]] std::vector<double> cumSum() const {
    double sum = 0;
    std::vector<double> cum_sum;
    for (const auto& w : weights) {
      sum += w;
      cum_sum.push_back(sum);
    }
    return cum_sum;
  }
};
/*********************************
 **         IID Belief          **
 ********************************/
class IIDBelief : public ObstacleBelief {
 public:
  IIDBelief(const ObstacleConfiguration& oc, const double noise, const std::vector<double>& bias)
      : ObstacleBelief(oc, noise, bias) {}

  void updateFreeSpace(const DenseGrid& new_free) override {}

  void updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) override {}

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
  std::unique_ptr<ChsBelief> cloneChsBelief() const {
    std::unique_ptr<ChsBelief> b = std::make_unique<ChsBelief>();
    b->known_free = known_free;
    b->chs = chs;
    return b;
  }

  std::unique_ptr<Belief> clone() const override { return cloneChsBelief(); }

  void viz(const GpuVoxelRvizVisualizer& viz) override {
    viz.vizChs(chs);

    viz.vizGrid(known_free, "known_free", makeColor(0, 0, 1, 0.1));
  }

  double calcProbFree(const DenseGrid& volume) override {
    PROFILE_START("CalcProbFreeCHS");
    double p_free = 1.0;
    for (auto& c : chs) {
      if (c.countOccupied() == 0) {
        // throw std::runtime_error("Empty CHS");
        continue;
      }
      p_free *= 1 - ((double)c.collideWith(&volume) / c.countOccupied());
    }
    PROFILE_RECORD("CalcProbFreeCHS");
    return p_free;
  }

  void updateFreeSpace(const DenseGrid& new_free) override {
    known_free.add(&new_free);
    for (auto& c : chs) {
      c.subtract(&new_free);
    }
  }

  void updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) override {
    addChs(robot, first_link_in_collision);
  }

  virtual void addChs(Robot& robot, const size_t first_link_in_collision) {
    auto link_occupancies = robot.getLinkOccupancies();
    if (first_link_in_collision >= link_occupancies.size()) {
      throw std::logic_error("Trying to add CHS, but no link collided");
    }

    DenseGrid new_chs;

    for (size_t i = first_link_in_collision; i < link_occupancies.size(); i++) {
      new_chs.add(&link_occupancies[i]);
    }
    new_chs.subtract(&known_free);
    chs.push_back(new_chs);
  }

  DenseGrid sampleState() const override {
    DenseGrid sampled_obstacles;
    for (const auto& c : chs) {
      c.copyRandomOccupiedElement(sampled_obstacles);
    }
    return sampled_obstacles;
  }
};

/*********************************
 **  Mixture Of Experts Belief  **
 ********************************/
class MoEBelief : public Belief {
 public:
  std::vector<double> weights;
  ChsBelief expert_chs;
  ObstacleBelief expert_particle;
  std::vector<Belief*> experts;
  std::vector<std::vector<double>> particle_prior;

  // public:
  //     MoEBelief(){}

 public:
  MoEBelief(const ObstacleConfiguration& oc, const double noise, const std::vector<double>& bias)
      : expert_particle(oc, noise, bias) {
    weights = std::vector<double>{1.0, 1.0};
    experts.push_back(&expert_chs);
    experts.push_back(&expert_particle);
    particle_prior = expert_particle.getParticleVectors();

    updateWeights();
    std::cout << "Weights are: " << PrettyPrint::PrettyPrint(weights) << "\n";
  }

  MoEBelief(ChsBelief chsb, ObstacleBelief obsb) : expert_chs(std::move(chsb)), expert_particle(std::move(obsb)) {}

  std::unique_ptr<Belief> clone() const override {
    std::cout << "Cloning MoE belief\n";
    std::unique_ptr<MoEBelief> b =
        std::make_unique<MoEBelief>(*expert_chs.cloneChsBelief(), *expert_particle.cloneObstacleBelief());
    // b->expert_particle = expert_particle;
    // b->expert_chs = expert_chs;
    b->weights = weights;
    b->experts.push_back(&(b->expert_chs));
    b->experts.push_back(&(b->expert_particle));
    b->particle_prior = particle_prior;
    return b;
  }

  static double kernelDensityLikelihood(const std::vector<std::vector<double>>& prior,
                                        const std::vector<std::vector<double>>& posterior,
                                        std::vector<double> posterior_weights, double bandwidth) {
    std::vector<double> new_weights;
    double posterior_mass = std::accumulate(posterior_weights.begin(), posterior_weights.end(), (double)0.0);

    if (posterior_mass <= 0.0) {
      return 0.0;
    }

    // std::cout << "posterior mass: " << posterior_mass << "\n";
    double normalizing = prior.size() * posterior_mass;
    // std::cout << "normalizing factor: " << normalizing << "\n";
    double total = 0;

    for (const auto& particle : posterior) {
      double w_i = 0;
      for (int i = 0; i < prior.size(); i++) {
        double density = 1;
        for (int j = 0; j < prior[i].size(); j++) {
          double d = particle[j] - prior[i][j];
          density *= arc_helpers::EvaluateGaussianPDF(0, bandwidth, d);
        }
        w_i += density * posterior_weights[i];
      }
      total += w_i / normalizing;
    }
    return total;
  }

  /* Returns a vector of the cumulative sum of the expert weights*/
  std::vector<double> cumSum() const {
    std::vector<double> cs;
    double sum = 0;
    for (const auto& w : weights) {
      sum += w;
      cs.push_back(sum);
    }
    return cs;
  }

  void viz(const GpuVoxelRvizVisualizer& viz) override{
    for (const auto& expert : experts) {
      expert->viz(viz);
    }
  }

  void updateWeights() {
    // weights[0] = weights[0] //CHS weights do not update
    // std::cout << "expert cum sum: " << expert_particle.cumSum().back() << "\n";
    weights[1] =
        kernelDensityLikelihood(particle_prior, expert_particle.getParticleVectors(), expert_particle.weights, 0.1);
  }

  double calcProbFree(const DenseGrid& volume) override {
    double normalizing = cumSum().back();
    double p = 0;
    for (int i = 0; i < experts.size(); i++) {
      p += (weights[i] / normalizing) * experts[i]->calcProbFree(volume);
    }
    return p;
  }

  void updateFreeSpace(const DenseGrid& new_free) override {
    for (auto & expert : experts) {
      expert->updateFreeSpace(new_free);
    }
    updateWeights();
  }

  void updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) override {
    for (auto & expert : experts) {
      expert->updateCollisionSpace(robot, first_link_in_collision);
    }
    updateWeights();
    std::cout << "Weights are: " << PrettyPrint::PrettyPrint(weights) << "\n";
  }

  DenseGrid sampleState() const override {
    std::vector<double> cum_sum = cumSum();
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_real_distribution<double> dist(0.0, cum_sum.back());
    double r = dist(rng);

    for (int i = 0; i < experts.size(); i++) {
      if (r <= cum_sum[i]) {
        return experts[i]->sampleState();
      }
    }
    throw std::logic_error("Sampling from mixture of experts did not find an expert");
  }
};
}  // namespace GVP

#endif
