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
class SparseGrid: private gpu_voxels::voxellist::ProbVoxelList
{
public:
    SparseGrid();

    SparseGrid(const DenseGrid &other);

    const gpu_voxels::voxellist::ProbVoxelList* getProbVoxelList() const;

    SparseGrid& operator=(const DenseGrid &other);

    void merge(const DenseGrid* other);

    float getVoxelSideLength() const override;

    uint64_t serializeSelf(std::vector<uint8_t>& buffer) const;
    
    bool deserializeSelf(std::vector<uint8_t>& buffer, uint64_t &buffer_index);

    bool writeToDisk(const std::string& path);

    bool readFromDisk(const std::string& path);

};




/***************************
 *   Dense Grid
 ****************************/

class DenseGrid : private gpu_voxels::voxelmap::ProbVoxelMap
{
public:
    DenseGrid();
    
    DenseGrid(const DenseGrid &other);

    explicit DenseGrid(const SparseGrid &other);

    const gpu_voxels::voxelmap::ProbVoxelMap* getProbVoxelMap() const;

    DenseGrid& operator=(const DenseGrid &other);

    void insertBox(const Vector3f &corner_min, const Vector3f &corner_max);

    void copyRandomOccupiedElement(DenseGrid& to) const;

    bool overlapsWith(const DenseGrid* other, float coll_threshold = 1.0) const;

    size_t collideWith(const DenseGrid* map);

    void subtract(const DenseGrid *other);

    void add(const DenseGrid *other);

    void copy(const DenseGrid *other);

    void merge(const SparseGrid *other);

    size_t countOccupied() const;
    
    void copyIthOccupied(const DenseGrid* other, unsigned long copy_index) const;

    void insertMetaPointCloud(const MetaPointCloud &meta_point_cloud, BitVoxelMeaning voxel_meaning) override;

    void clearMap() override;

    void insertPointCloud(const std::vector<Vector3f> &point_cloud, const BitVoxelMeaning voxel_meaning) override;

    void insertPointCloud(const PointCloud &pointcloud, const BitVoxelMeaning voxel_meaning) override;

    std::vector<Vector3f> getOccupiedCenters() const;
    std::vector<Vector3ui> getOccupiedCoords() const;

    float getVoxelSideLength() const override;

    Vector3ui getDimensions() const override;

    bool writeToDisk(const std::string& path);

    bool readFromDisk(const std::string& path);
};


#endif
