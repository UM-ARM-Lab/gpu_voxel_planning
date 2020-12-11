#ifndef PROB_MAP_HPP
#define PROB_MAP_HPP

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>

#define GRID_X_DIM 256
#define GRID_Y_DIM 256
#define GRID_Z_DIM 256
#define VOXEL_SIDE_LENGTH 0.02

class DenseGrid;

/***************************
 *   Sparse Grid
 ****************************/
class SparseGrid : public gpu_voxels::voxellist::ProbVoxelList {
 public:
  SparseGrid();
  explicit SparseGrid(const DenseGrid& other);
  SparseGrid& operator=(const DenseGrid& other);
  void merge(const DenseGrid* other);
};

/***************************
 *   Dense Grid
 ****************************/

class DenseGrid : public gpu_voxels::voxelmap::ProbVoxelMap {
 public:
  DenseGrid();
  DenseGrid(const DenseGrid& other);
  explicit DenseGrid(const SparseGrid& other);
  DenseGrid& operator=(const DenseGrid& other);
  void insertBox(const Vector3f& corner_min, const Vector3f& corner_max);
  void copyRandomOccupiedElement(DenseGrid& to) const;
  bool overlapsWith(const DenseGrid* other, float coll_threshold = 1.0) const;
  size_t collideWith(const DenseGrid* map);
  void subtract(const DenseGrid* other);
  void add(const DenseGrid* other);
  void copy(const DenseGrid* other);
  void merge(const SparseGrid* other);
  void copyIthOccupied(const DenseGrid* other, unsigned long copy_index) const;
};

#endif
