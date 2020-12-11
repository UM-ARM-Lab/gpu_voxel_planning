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
class SparseGrid : private gpu_voxels::voxellist::ProbVoxelList {
 public:
  SparseGrid();
  explicit SparseGrid(const DenseGrid& other);
  const gpu_voxels::voxellist::ProbVoxelList* getProbVoxelList() const;
  SparseGrid& operator=(const DenseGrid& other);
  void merge(const DenseGrid* other);

  using gpu_voxels::voxellist::ProbVoxelList::getVoxelSideLength;
  using gpu_voxels::voxellist::ProbVoxelList::serializeSelf;
  using gpu_voxels::voxellist::ProbVoxelList::deserializeSelf;
  using gpu_voxels::voxellist::ProbVoxelList::writeToDisk;
  using gpu_voxels::voxellist::ProbVoxelList::readFromDisk;
};

/***************************
 *   Dense Grid
 ****************************/

class DenseGrid : private gpu_voxels::voxelmap::ProbVoxelMap {
 public:
  DenseGrid();
  DenseGrid(const DenseGrid& other);
  explicit DenseGrid(const SparseGrid& other);
  const gpu_voxels::voxelmap::ProbVoxelMap* getProbVoxelMap() const;
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

  using gpu_voxels::voxelmap::ProbVoxelMap::getVoxelSideLength;
  using gpu_voxels::voxelmap::ProbVoxelMap::getOccupiedCenters;
  using gpu_voxels::voxelmap::ProbVoxelMap::getOccupiedCoords;
  using gpu_voxels::voxelmap::ProbVoxelMap::getDimensions;
  using gpu_voxels::voxelmap::ProbVoxelMap::clearMap;
  using gpu_voxels::voxelmap::ProbVoxelMap::countOccupied;
  using gpu_voxels::voxelmap::ProbVoxelMap::insertPointCloud;
  using gpu_voxels::voxelmap::ProbVoxelMap::insertMetaPointCloud;
  using gpu_voxels::voxelmap::ProbVoxelMap::writeToDisk;
  using gpu_voxels::voxelmap::ProbVoxelMap::readFromDisk;
};

#endif
