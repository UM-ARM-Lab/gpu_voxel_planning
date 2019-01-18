#ifndef GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define GPU_VOXEL_RVIZ_VISUALIZATION_HPP

#include <visualization_msgs/Marker.h>
#include "prob_map.hpp"
#include "state.hpp"
#include <ros/ros.h>


visualization_msgs::Marker visualizeProbGrid(const ProbGrid &grid,
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



class GpuVoxelRvizVisualizer
{
public:
    ros::Publisher grid_pub;
    std::string global_frame = "gpu_voxel_frame";

    GpuVoxelRvizVisualizer(ros::NodeHandle &n)
    {
        grid_pub = n.advertise<visualization_msgs::Marker>("grid", 10);
    }

    void vizGrid(const ProbGrid &grid, const std::string &ns, const std_msgs::ColorRGBA &color)
    {
        grid_pub.publish(visualizeProbGrid(grid, global_frame, ns, color));
    }

    void vizChs(const ProbGrid &grid, const std::string &ns = "chs")
    {
        std_msgs::ColorRGBA color;
        color.a = 0.3;
        color.r = 1.0;
        vizGrid(grid, ns, color);
    }

    void vizState(const GVP::State &s)
    {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        grid_pub.publish(visualizeProbGrid(s.passive_robot.occupied_space, global_frame,
                                           "passive_robot", color));
        grid_pub.publish(visualizeProbGrid(s.known_obstacles, global_frame,
                                           "known_obstacles", color));

        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;
        grid_pub.publish(visualizeProbGrid(s.active_robot.occupied_space, global_frame,
                                           "active_robot", color));

    }
};



#endif
