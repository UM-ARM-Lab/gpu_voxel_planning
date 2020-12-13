//
// Created by bsaund on 12/13/20.
//
#include <gpu_voxel_planning/beliefs/beliefs.hpp>

using namespace GVP;

/******************************************************************************
 * Obstacle Belief
 ******************************************************************************/

GVP::ObstacleBelief::ObstacleBelief(const ObstacleConfiguration& oc, const double noise,
                                    const std::vector<double>& bias) {
  int num_samples = 10;  // HARDCODED PARAM
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

std::unique_ptr<ObstacleBelief> GVP::ObstacleBelief::cloneObstacleBelief() const {
  std::cout << "cloning obstacle belief\n";
  std::unique_ptr<ObstacleBelief> b = std::make_unique<ObstacleBelief>();
  b->particles = particles;
  b->weights = weights;
  return b;
}

void GVP::ObstacleBelief::addElem(const GVP::ObstacleConfiguration& obs, double weight) {
  particles.push_back(obs);
  weights.push_back(weight);
}

double GVP::ObstacleBelief::calcProbFree(const DenseGrid& volume) {
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

void GVP::ObstacleBelief::updateFreeSpace(const DenseGrid& new_free) {
  PROFILE_START("Obstacle_belief_update_free");
  for (int i = 0; i < particles.size(); i++) {
    if (weights[i] == 0) {
      continue;
    }
    // heuristic for near collisions
    if (particles[i].occupied.collideWith(&new_free) > 0){
      weights[i] = 0;
    }
  }
  PROFILE_RECORD("Obstacle_belief_update_free");
}

std::vector<std::vector<double>> GVP::ObstacleBelief::getParticleVectors() const {
  std::vector<std::vector<double>> pvs;
  for (const auto& particle : particles) {
    pvs.push_back(particle.asParticle());
  }
  return pvs;
}

std::vector<double> ObstacleBelief::kernelDensityEstimate(std::vector<std::vector<double>> prior,
                                                          double bandwidth) const {
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

void ObstacleBelief::updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) {
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
DenseGrid ObstacleBelief::sampleState() const {
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

void ObstacleBelief::viz(const GpuVoxelRvizVisualizer& viz) {
  std::vector<DenseGrid*> grids;
  std::vector<double> alphas;
  for (int i = 0; i < particles.size(); i++) {
    if(weights[i] == 0){
      continue;
    }
    alphas.push_back(std::max(weights[i] / cumSum().back(), 1.0 / 100));
    grids.emplace_back(&particles[i].occupied);
  }
  viz.vizGrids(grids, alphas, "belief_obstacles");
}
std::vector<double> ObstacleBelief::cumSum() const {
  double sum = 0;
  std::vector<double> cum_sum;
  for (const auto& w : weights) {
    sum += w;
    cum_sum.push_back(sum);
  }
  return cum_sum;
}

/***************************************************************
 * CHS Belief
 ***************************************************************/

std::unique_ptr<ChsBelief> ChsBelief::cloneChsBelief() const {
  std::unique_ptr<ChsBelief> b = std::make_unique<ChsBelief>();
  b->known_free = known_free;
  b->chs = chs;
  return b;
}
void ChsBelief::viz(const GpuVoxelRvizVisualizer& viz) {
  viz.vizChs(chs);
  viz.vizGrid(known_free, "known_free", makeColor(0, 0, 1, 0.1));
}
double ChsBelief::calcProbFree(const DenseGrid& volume) {
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

void ChsBelief::updateFreeSpace(const DenseGrid& new_free) {
  known_free.add(&new_free);
  for (auto& c : chs) {
    c.subtract(&new_free);
  }
}

void ChsBelief::updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) {
  addChs(robot, first_link_in_collision);
}

void ChsBelief::addChs(Robot& robot, const size_t first_link_in_collision) {
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
DenseGrid ChsBelief::sampleState() const {
  DenseGrid sampled_obstacles;
  for (const auto& c : chs) {
    c.copyRandomOccupiedElement(sampled_obstacles);
  }
  return sampled_obstacles;
}

/************************************************************
 * MOE Belief
 ************************************************************/

MoEBelief::MoEBelief(const ObstacleConfiguration& oc, const double noise, const std::vector<double>& bias)
    : expert_particle(oc, noise, bias) {
  weights = std::vector<double>{1.0, 1.0};
  experts.push_back(&expert_chs);
  experts.push_back(&expert_particle);
  particle_prior = expert_particle.getParticleVectors();

  updateWeights();
  std::cout << "Weights are: " << PrettyPrint::PrettyPrint(weights) << "\n";
}

std::unique_ptr<Belief> MoEBelief::clone() const {
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
double MoEBelief::kernelDensityLikelihood(const std::vector<std::vector<double>>& prior,
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
std::vector<double> MoEBelief::cumSum() const {
  std::vector<double> cs;
  double sum = 0;
  for (const auto& w : weights) {
    sum += w;
    cs.push_back(sum);
  }
  return cs;
}
void MoEBelief::viz(const GpuVoxelRvizVisualizer& viz) {
  for (const auto& expert : experts) {
    expert->viz(viz);
  }
}
void MoEBelief::updateWeights() {
  // weights[0] = weights[0] //CHS weights do not update
  // std::cout << "expert cum sum: " << expert_particle.cumSum().back() << "\n";
  weights[1] =
      kernelDensityLikelihood(particle_prior, expert_particle.getParticleVectors(), expert_particle.weights, 0.1);
}
double MoEBelief::calcProbFree(const DenseGrid& volume) {
  double normalizing = cumSum().back();
  double p = 0;
  for (int i = 0; i < experts.size(); i++) {
    p += (weights[i] / normalizing) * experts[i]->calcProbFree(volume);
  }
  return p;
}
void MoEBelief::updateFreeSpace(const DenseGrid& new_free) {
  for (auto& expert : experts) {
    expert->updateFreeSpace(new_free);
  }
  updateWeights();
}
void MoEBelief::updateCollisionSpace(Robot& robot, const size_t first_link_in_collision) {
  for (auto& expert : experts) {
    expert->updateCollisionSpace(robot, first_link_in_collision);
  }
  updateWeights();
  std::cout << "Weights are: " << PrettyPrint::PrettyPrint(weights) << "\n";
}
DenseGrid MoEBelief::sampleState() const {
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
