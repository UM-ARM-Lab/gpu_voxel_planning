#ifndef GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define GPU_VOXEL_RVIZ_VISUALIZATION_HPP

#include <visualization_msgs/Marker.h>
#include "prob_map.hpp"
#include "state.hpp"
#include "scenarios.hpp"
#include <ros/ros.h>
#include "urdf_model.hpp"


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

inline visualization_msgs::Marker visualizePoint(const Eigen::Vector3d pos,
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

    return point_marker;
}

inline visualization_msgs::Marker visualize3DPath(const std::vector<Eigen::Vector3d> path,
                                                  const std::string& frame,
                                                  const std::string& ns,
                                                  const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker point_marker;
    point_marker.ns = ns;
    point_marker.header.frame_id = frame;
    point_marker.type = visualization_msgs::Marker::LINE_STRIP;
    point_marker.color = color;
    point_marker.scale.x = 0.01;
    point_marker.orientation.w = 1.0;

    for(const Eigen::Vector3d& point:path)
    {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        point_marker.points.push_back(p);
    }
    return point_marker;
}



static inline std_msgs::ColorRGBA makeColor(double r, double g, double b, double a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;     color.g = g;    color.b = b;     color.a = a;
    return color;
}

class GpuVoxelRvizVisualizer
{
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
        ee_path_pub = n.advertise<visualization_msgs::Marker>("ee_path", 10);
    }

    void vizEEPosition(const std::vector<double> config)
    {
        ee_path_pub.publish(visualizePoint(urdf.getEEPosition(config), "victor_root", "position",
                                           makeColor(0.0, 0.0, 1.0)));
    }

    void vizEEPath(const std::vector<GVP::VictorRightArmConfig> path_config)
    {
        std::vector<Eigen::Vector3d> path_3d;

        for(auto config: path_config)
        {
            path_3d.push_back(urdf.getEEPosition(config.asVector()));
        }
        
        ee_path_pub.publish(visualize3DPath(path_3d, "victor_root", "EE_path",
                                           makeColor(0.0, 0.0, 1.0)));
    }

    void vizGrid(const DenseGrid &grid, const std::string &ns, const std_msgs::ColorRGBA &color)
    {
        grid_pub.publish(visualizeDenseGrid(grid, global_frame, ns, color));
    }

    void vizChs(const std::vector<DenseGrid> &chss, const std::string &ns = "chs")
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

    void vizState(const GVP::State &s)
    {
        grid_pub.publish(visualizeDenseGrid(s.known_obstacles, global_frame,
                                           "known_obstacles", makeColor(0,0,0,1)));

        std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
        grid_pub.publish(visualizeDenseGrid(s.robot_self_collide_obstacles, global_frame,
                                           "passive_robot", robot_color));
        grid_pub.publish(visualizeDenseGrid(s.robot.occupied_space, global_frame,
                                           "active_robot", robot_color));
        vizChs(s.chs);
    }

    void vizScenario(const GVP::Scenario &s)
    {
        vizState(s.getState());   
    }

    void vizScenario(const GVP::SimulationScenario &s)
    {
        grid_pub.publish(visualizeDenseGrid(s.getTrueObstacles(), global_frame,
                                           "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5)));
        vizState(s.getState());   
    }
};



#endif
