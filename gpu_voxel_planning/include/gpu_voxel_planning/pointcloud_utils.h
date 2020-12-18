//
// Created by bsaund on 12/18/20.
//

#ifndef GPU_VOXEL_PLANNING_POINTCLOUD_UTILS_H
#define GPU_VOXEL_PLANNING_POINTCLOUD_UTILS_H

#include <sensor_msgs/point_cloud2_iterator.h>

#include "gpu_voxel_planning/maps/prob_map.hpp"

inline std::vector<Vector3f> toPointsVector(const sensor_msgs::PointCloud2& pt_cloud) {
  if (pt_cloud.header.frame_id != "gpu_voxel_world") {
    throw std::runtime_error("Invalid frame ID " + pt_cloud.header.frame_id + ". Must be gpu_voxel_world");
  }

  std::vector<Vector3f> points;
  for (sensor_msgs::PointCloud2ConstIterator<float> it(pt_cloud, "x"); it != it.end(); ++it) {
    points.emplace_back(it[0], it[1], it[2]);
  }
  return points;
}

inline sensor_msgs::PointCloud2 toMsg(const DenseGrid& grid) {
  sensor_msgs::PointCloud2 msg;
  auto pts = grid.getOccupiedCenters();
  msg.header.frame_id = "gpu_voxel_world";
  msg.height = 1;
  msg.width = pts.size();
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[1].name = "y";
  msg.fields[2].name = "z";
  int offset = 0;
  for (auto& field : msg.fields) {
    field.offset = offset;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    offset += 4;
  }
  msg.point_step = offset;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.point_step * msg.width);
  msg.is_bigendian = false;
  msg.is_dense = false;

  for (size_t cp = 0; cp < pts.size(); cp++) {
    memcpy(&msg.data[cp * msg.point_step + msg.fields[0].offset], &pts[cp].x, sizeof(float));
    memcpy(&msg.data[cp * msg.point_step + msg.fields[1].offset], &pts[cp].y, sizeof(float));
    memcpy(&msg.data[cp * msg.point_step + msg.fields[2].offset], &pts[cp].z, sizeof(float));
  }
  return msg;
}

#endif  // GPU_VOXEL_PLANNING_POINTCLOUD_UTILS_H
