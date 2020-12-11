#include "gpu_voxel_planning/maps/prob_map.hpp"

#include <random>

/********************
 ** Sparse Grid
 *******************/
SparseGrid::SparseGrid()
    : gpu_voxels::voxellist::ProbVoxelList(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM), VOXEL_SIDE_LENGTH,
                                           MT_PROBAB_VOXELLIST) {}

SparseGrid::SparseGrid(const DenseGrid& other)
    : gpu_voxels::voxellist::ProbVoxelList(other.getDimensions(), other.getVoxelSideLength(), MT_PROBAB_VOXELLIST) {
  merge(&other);
}

SparseGrid& SparseGrid::operator=(const DenseGrid& other) {
  merge(&other);
  return *this;
}

void SparseGrid::merge(const DenseGrid* other) {
  gpu_voxels::voxellist::ProbVoxelList::merge(other);
}

/*******************
 ** Dense Grid
 ******************/
DenseGrid::DenseGrid()
    : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM), VOXEL_SIDE_LENGTH,
                                         MT_PROBAB_VOXELMAP) {}

DenseGrid::DenseGrid(const DenseGrid& other)
    : gpu_voxels::voxelmap::ProbVoxelMap(other.getDimensions(), other.getVoxelSideLength(), MT_PROBAB_VOXELMAP) {
  copy(&other);
}

DenseGrid::DenseGrid(const SparseGrid& other)
    : gpu_voxels::voxelmap::ProbVoxelMap(Vector3ui(GRID_X_DIM, GRID_Y_DIM, GRID_Z_DIM), other.getVoxelSideLength(),
                                         MT_PROBAB_VOXELMAP) {
  merge(&other);
}

DenseGrid& DenseGrid::operator=(const DenseGrid& other) {
  copy(&other);
  return *this;
}

void DenseGrid::insertBox(const Vector3f& corner_min, const Vector3f& corner_max) {
  float delta = VOXEL_SIDE_LENGTH / 2.0;
  insertPointCloud(geometry_generation::createBoxOfPoints(corner_min, corner_max, delta), eBVM_OCCUPIED);
}

void DenseGrid::copyRandomOccupiedElement(DenseGrid& to) const {
  size_t num_occupied = countOccupied();
  std::mt19937 generator;
  generator.seed(std::random_device()());
  std::uniform_int_distribution<unsigned long> dist(0, num_occupied);
  unsigned long rand_index = dist(generator);
  copyIthOccupied(&to, rand_index);
}

bool DenseGrid::overlapsWith(const DenseGrid* other, float coll_threshold) const {
  return voxelmap::ProbVoxelMap::overlapsWith(other, coll_threshold);
}

size_t DenseGrid::collideWith(const DenseGrid* map) { return voxelmap::ProbVoxelMap::collideWith(map); }

void DenseGrid::subtract(const DenseGrid* other) { voxelmap::ProbVoxelMap::subtract(other); }

void DenseGrid::add(const DenseGrid* other) { voxelmap::ProbVoxelMap::add(other); }

void DenseGrid::copy(const DenseGrid* other) { voxelmap::ProbVoxelMap::copy(other); }

void DenseGrid::merge(const SparseGrid* other) { voxelmap::ProbVoxelMap::merge(other); }

void DenseGrid::copyIthOccupied(const DenseGrid* other, unsigned long copy_index) const {
  voxelmap::ProbVoxelMap::copyIthOccupied(other, copy_index);
}
