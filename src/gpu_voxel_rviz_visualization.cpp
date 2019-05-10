#include "ros_interface/gpu_voxel_rviz_visualization.hpp"

using namespace GVP;

visualization_msgs::Marker GVP::visualizeDenseGrid(const DenseGrid &grid,
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


visualization_msgs::MarkerArray GVP::visualizePoint(const Eigen::Vector3d pos,
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


visualization_msgs::MarkerArray GVP::visualize3DPath(const std::vector<Eigen::Vector3d> path,
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




GVP::GpuVoxelRvizVisualizer::GpuVoxelRvizVisualizer(ros::NodeHandle &n)
{
    grid_pub = n.advertise<visualization_msgs::Marker>("grid", 10);
    chs_pub = n.advertise<visualization_msgs::Marker>("chs", 10);
    ee_path_pub = n.advertise<visualization_msgs::MarkerArray>("ee_path", 10);
}

void GVP::GpuVoxelRvizVisualizer::vizEEPosition(const std::vector<double> config)
{
    ee_path_pub.publish(visualizePoint(urdf.getEEPosition(config), global_frame, "position",
                                       makeColor(0.0, 0.0, 1.0)));
}

void GVP::GpuVoxelRvizVisualizer::vizGrid(const DenseGrid& grid, const std::string& name,
                                          const std_msgs::ColorRGBA& color) const
{
    grid_pub.publish(visualizeDenseGrid(grid, global_frame, name, color));
}

void GVP::GpuVoxelRvizVisualizer::vizChs(const std::vector<DenseGrid> &chss,
                                         const std::string &ns) const
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




