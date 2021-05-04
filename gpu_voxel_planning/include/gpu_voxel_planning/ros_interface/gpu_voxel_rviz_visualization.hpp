#ifndef GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define GPU_VOXEL_RVIZ_VISUALIZATION_HPP

#include <ros/ros.h>
#include <rviz_voxelgrid_visuals_msgs/SparseVoxelgridStamped.h>
#include <rviz_voxelgrid_visuals_msgs/VoxelgridStamped.h>
#include <visualization_msgs/Marker.h>

#include "gpu_voxel_planning/maps/prob_map.hpp"
#include "gpu_voxel_planning/robot/urdf_model.hpp"

namespace GVP {
visualization_msgs::Marker visualizeDenseGrid(const DenseGrid& grid, const std::string& global_frame,
                                              const std::string& ns, const std_msgs::ColorRGBA& color);

visualization_msgs::Marker visualizeDenseGrid(const std::vector<Vector3f>& centers, float side_length,
                                              const std::string& global_frame, const std::string& ns,
                                              const std_msgs::ColorRGBA& color);

visualization_msgs::MarkerArray visualizePoint(const Eigen::Vector3d& pos, const std::string& frame,
                                               const std::string& ns, int id, const std_msgs::ColorRGBA& color);

visualization_msgs::MarkerArray visualize3DPath(const std::vector<Eigen::Vector3d>& path, const std::string& frame,
                                                const std::string& ns, int id, const std_msgs::ColorRGBA& color);

rviz_voxelgrid_visuals_msgs::SparseVoxelgridStamped denseGridToMsg(const DenseGrid& g, const std::string& frame);
rviz_voxelgrid_visuals_msgs::SparseVoxelgridStamped denseGridsToMsg(const std::vector<DenseGrid*>& g,
                                                                    const std::vector<double>& alphas,
                                                                    const std::string& frame);

inline std_msgs::ColorRGBA makeColor(double r, double g, double b, double a = 1.0) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

class GpuVoxelRvizVisualizer {
 public:
  std::string topic_prefix = "voxelgrid/";
  std::vector<std::string> grid_names{"known_obstacles", "unknown_obstacles", "active_robot",    "passive_robot",
                                      "known_free",      "true_obstacles",    "belief_obstacles", "swept_edge"};
  std::map<std::string, ros::Publisher> grid_pubs;
  //  ros::Publisher grid_pub;
  ros::Publisher chs_pub;
  ros::Publisher ee_path_pub;
  std::string global_frame = "gpu_voxel_world";
  VictorKinematics urdf;

 private:
  std::vector<Eigen::Vector3d> configPathTo3DPath(const std::vector<GVP::VictorRightArmConfig>& path_config) {
    std::vector<Eigen::Vector3d> path_3d;
    for (const auto& config : path_config) {
      path_3d.push_back(urdf.getEEPosition(config.asVector()));
    }
    return path_3d;
  }

 public:
  explicit GpuVoxelRvizVisualizer(ros::NodeHandle& n);

  void vizEEPosition(const std::vector<double>& config, const std_msgs::ColorRGBA& color, int id = 0);

  virtual void vizEEPath(const std::vector<GVP::VictorRightArmConfig>& path_config, const std::string& path_name,
                         int id, const std_msgs::ColorRGBA& color) {
    std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(path_config);
    ee_path_pub.publish(visualize3DPath(path_3d, global_frame, path_name, id, color));
  }

  virtual void vizGrid(const DenseGrid& grid, const std::string& name, const std_msgs::ColorRGBA& color) const;
  virtual void vizGrid(const DenseGrid& grid, const std::string& name) const;
  virtual void clearGrid(const std::string& name) const;

  virtual void clearAllGrids() const;

  virtual void vizGrids(const std::vector<DenseGrid*>& grids, const std::vector<double>& alphas,
                        const std::string& name) const;

  //  void vizGrid(const std::vector<Vector3f>& centers, float side_length, const std::string& name,
  //               const std_msgs::ColorRGBA& color) const;

  // void vizEEGraph(const HaltonGraph &g)
  // {
  //     visualization_msgs::MarkerArray marker_array;
  //     int id = 0;
  //     for(const auto n:g.getNodes())
  //     {
  //         for(const auto e:n.getOutEdges())
  //         {
  //             if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
  //             {
  //                 continue;
  //             }
  //             std_msgs::ColorRGBA color = makeColor(0.0, 0.0, 0.0);
  //             std::string ns = "valid";
  //             if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
  //             {
  //                 color = makeColor(1.0, 0, 0);
  //                 ns = "invalid";
  //             }

  //             GVP::VictorRightArmConfig q_start(g.getNode(e.getFromIndex()).getValue());
  //             GVP::VictorRightArmConfig q_goal(g.getNode(e.getToIndex()).getValue());

  //             GVP::Path edge_config = interpolate(q_start, q_goal, 0.01);

  //             std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(edge_config);

  //             auto arr = visualize3DPath(path_3d, global_frame, ns, color);
  //             arr.markers[0].id = id++;
  //             marker_array.markers.push_back(arr.markers[0]);
  //         }
  //     }
  //     ee_path_pub.publish(marker_array);
  // }

  // void vizEESDGraph(const SDRoadmap &g)
  // {
  //     visualization_msgs::MarkerArray marker_array;
  //     int id = 0;
  //     for(const auto n:g.getNodes())
  //     {
  //         // if(DepthNode(n.getValue()).depth != depth)
  //         // {
  //         //     continue;
  //         // }
  //         for(const auto e:n.getOutEdges())
  //         {
  //             if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
  //             {
  //                 continue;
  //             }
  //             std_msgs::ColorRGBA color = makeColor(0.0, 0.0, 0.0);
  //             std::string ns = "valid";
  //             if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
  //             {
  //                 color = makeColor(1.0, 0, 0);
  //                 ns = "invalid";
  //             }

  //             GVP::VictorRightArmConfig q_start(g.getNodeValue(e.getFromIndex()).q);
  //             GVP::VictorRightArmConfig q_goal(g.getNodeValue(e.getToIndex()).q);

  //             GVP::Path edge_config = interpolate(q_start, q_goal, 0.01);

  //             std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(edge_config);

  //             auto arr = visualize3DPath(path_3d, global_frame, ns, color);
  //             arr.markers[0].id = id++;
  //             marker_array.markers.push_back(arr.markers[0]);
  //         }
  //     }
  //     ee_path_pub.publish(marker_array);
  // }

  void vizChs(const std::vector<DenseGrid>& chss, const std::string& ns = "chs") const;

 protected:
  GpuVoxelRvizVisualizer() : urdf(true){};  // used for mocking
};

}  // namespace GVP

#endif
