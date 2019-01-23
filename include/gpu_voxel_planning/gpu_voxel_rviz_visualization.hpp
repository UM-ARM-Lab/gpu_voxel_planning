#ifndef GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define GPU_VOXEL_RVIZ_VISUALIZATION_HPP

#include <visualization_msgs/Marker.h>
#include "prob_map.hpp"
#include "state.hpp"
#include "scenarios.hpp"
#include <ros/ros.h>


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



static inline std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
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
    std::string global_frame = "gpu_voxel_frame";

    GpuVoxelRvizVisualizer(ros::NodeHandle &n)
    {
        grid_pub = n.advertise<visualization_msgs::Marker>("grid", 10);
        chs_pub = n.advertise<visualization_msgs::Marker>("chs", 10);
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
