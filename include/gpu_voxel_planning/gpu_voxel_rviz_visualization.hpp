#ifndef GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define GPU_VOXEL_RVIZ_VISUALIZATION_HPP

#include <visualization_msgs/Marker.h>
#include "maps/prob_map.hpp"
// #include "state.hpp"
// #include "scenarios.hpp"
#include <ros/ros.h>
#include "urdf_model.hpp"
#include "path_utils_addons.hpp"
// #include <graph_planner/increasing_density_halton.hpp>
// #include "strategies/graph_search_strategies.hpp"
#include "strategies/victor_selective_densification.hpp"

inline visualization_msgs::Marker visualizeDenseGrid(const DenseGrid &grid,
                                                     const std::string& global_frame,
                                                     const std::string& ns,
                                                     const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker occupied_marker;
    auto centers = grid.getOccupiedCenters();
    occupied_marker.points.resize(centers.size());
    occupied_marker.ns = ns;
    occupied_marker.type = visualization_msgs::Marker::CUBE_LIST;
    occupied_marker.header.frame_id = global_frame;
    occupied_marker.color = color;
    
    occupied_marker.scale.x = grid.getVoxelSideLength();
    occupied_marker.scale.y = grid.getVoxelSideLength();
    occupied_marker.scale.z = grid.getVoxelSideLength();


    for(const auto &c:centers)
    {
        geometry_msgs::Point p;
        p.x = c.x; p.y = c.y; p.z = c.z;
        occupied_marker.points.push_back(p);
    }
    return occupied_marker;
}

inline visualization_msgs::MarkerArray visualizePoint(const Eigen::Vector3d pos,
                                                 const std::string& frame,
                                                 const std::string& ns,
                                                 const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker point_marker;
    point_marker.ns = ns;
    point_marker.header.frame_id = frame;
    point_marker.type = visualization_msgs::Marker::CUBE;
    point_marker.color = color;
    point_marker.scale.x = 0.1;
    point_marker.scale.y = 0.1;
    point_marker.scale.z = 0.1;

    geometry_msgs::Point p;
    point_marker.pose.position.x = pos.x();
    point_marker.pose.position.y = pos.y();
    point_marker.pose.position.z = pos.z();
    point_marker.pose.orientation.w = 1.0;

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(point_marker);
    return arr;
}

inline visualization_msgs::MarkerArray visualize3DPath(const std::vector<Eigen::Vector3d> path,
                                                  const std::string& frame,
                                                  const std::string& ns,
                                                  const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker path_marker;
    path_marker.ns = ns;
    path_marker.header.frame_id = frame;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.color = color;
    path_marker.scale.x = 0.01;
    path_marker.pose.orientation.w = 1.0;

    for(const Eigen::Vector3d& point:path)
    {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        path_marker.points.push_back(p);
    }
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(path_marker);
    return arr;
}





static inline std_msgs::ColorRGBA makeColor(double r, double g, double b, double a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;     color.g = g;    color.b = b;     color.a = a;
    return color;
}





class GpuVoxelRvizVisualizer
{
private:
    std::vector<Eigen::Vector3d> configPathTo3DPath(std::vector<GVP::VictorRightArmConfig> path_config)
    {
        std::vector<Eigen::Vector3d> path_3d;
        for(auto config: path_config)
        {
            path_3d.push_back(urdf.getEEPosition(config.asVector()));
        }
        return path_3d;
    }
    
public:
    ros::Publisher grid_pub;
    ros::Publisher chs_pub;
    ros::Publisher ee_path_pub;
    std::string global_frame = "gpu_voxel_world";
    VictorKinematics urdf;

    GpuVoxelRvizVisualizer(ros::NodeHandle &n)
    {
        grid_pub = n.advertise<visualization_msgs::Marker>("grid", 10);
        chs_pub = n.advertise<visualization_msgs::Marker>("chs", 10);
        ee_path_pub = n.advertise<visualization_msgs::MarkerArray>("ee_path", 10);
    }

    void vizEEPosition(const std::vector<double> config)
    {
        ee_path_pub.publish(visualizePoint(urdf.getEEPosition(config), global_frame, "position",
                                           makeColor(0.0, 0.0, 1.0)));
    }

    void vizEEPath(const std::vector<GVP::VictorRightArmConfig> path_config,
                   std::string path_name)
    {
        std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(path_config);
        ee_path_pub.publish(visualize3DPath(path_3d, global_frame, path_name,
                                           makeColor(0.0, 0.0, 1.0)));
    }

    void vizGrid(const DenseGrid& grid, const std::string& name, const std_msgs::ColorRGBA& color)
    {
        grid_pub.publish(visualizeDenseGrid(grid, global_frame, name, color));
    }

    void vizEEGraph(const HaltonGraph &g)
    {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for(const auto n:g.getNodes())
        {
            for(const auto e:n.getOutEdges())
            {
                if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                {
                    continue;
                }
                std_msgs::ColorRGBA color = makeColor(0.0, 0.0, 0.0);
                std::string ns = "valid";
                if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
                {
                    color = makeColor(1.0, 0, 0);
                    ns = "invalid";
                }

                
                GVP::VictorRightArmConfig q_start(g.getNode(e.getFromIndex()).getValue());
                GVP::VictorRightArmConfig q_goal(g.getNode(e.getToIndex()).getValue());

                GVP::Path edge_config = interpolate(q_start, q_goal, 0.01);
                
                std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(edge_config);

                auto arr = visualize3DPath(path_3d, global_frame, ns, color);
                arr.markers[0].id = id++;
                marker_array.markers.push_back(arr.markers[0]);
            }
        }
        ee_path_pub.publish(marker_array);
    }

    
    void vizEESDGraph(const SDRoadmap &g)
    {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for(const auto n:g.getNodes())
        {
            // if(DepthNode(n.getValue()).depth != depth)
            // {
            //     continue;
            // }
            for(const auto e:n.getOutEdges())
            {
                if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                {
                    continue;
                }
                std_msgs::ColorRGBA color = makeColor(0.0, 0.0, 0.0);
                std::string ns = "valid";
                if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
                {
                    color = makeColor(1.0, 0, 0);
                    ns = "invalid";
                }

                
                GVP::VictorRightArmConfig q_start(g.getNodeValue(e.getFromIndex()).q);
                GVP::VictorRightArmConfig q_goal(g.getNodeValue(e.getToIndex()).q);

                GVP::Path edge_config = interpolate(q_start, q_goal, 0.01);
                
                std::vector<Eigen::Vector3d> path_3d = configPathTo3DPath(edge_config);

                auto arr = visualize3DPath(path_3d, global_frame, ns, color);
                arr.markers[0].id = id++;
                marker_array.markers.push_back(arr.markers[0]);
            }
        }
        ee_path_pub.publish(marker_array);
    }

    

    void vizGrid(const DenseGrid &grid, const std::string &ns, const std_msgs::ColorRGBA &color) const
    {
        grid_pub.publish(visualizeDenseGrid(grid, global_frame, ns, color));
    }

    void vizChs(const std::vector<DenseGrid> &chss, const std::string &ns = "chs") const
    {
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        color.r = 1.0;
        for(size_t i=0; i<chss.size(); i++)
        {
            std::ostringstream ss;
            ss << ns << "_" << i;
            chs_pub.publish(visualizeDenseGrid(chss[i], global_frame, ss.str(), color));
        }
    }
};



#endif
