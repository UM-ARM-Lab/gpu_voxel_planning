#include "prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>

#include <ros/ros.h>
#include "gpu_voxel_rviz_visualization.hpp"


void checkNoGpuMemoryLeaks()
{
// Run nvidia-smi and make sure gpu memory does not grow
    std::cout << "Starting trial\n";


    int counter = 0;

    for(int i=0; i<100000; i++)
    {
        PROFILE_START("Copy and Collide");
        ProbGrid g1;
        PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                              Vector3f(2.0,1.0,1.2),
                                                              VOXEL_SIDE_LENGTH/2));
        g1.insertPointCloud(box, PROB_OCCUPIED);

        ProbGrid g2(g1);

        if(g1.collideWith(&g2))
        {
            counter++;
        }
        PROFILE_RECORD("Copy and Collide");
    }

    PROFILE_PRINT_SUMMARY_FOR_SINGLE("Copy and Collide");
    std::cout << "Ran trials and found " << counter << " collision\n";

}

int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;
    GpuVoxelRvizVisualizer viz(n);

    ProbGrid g1;
    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                          Vector3f(1.1,1.0,1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box, PROB_OCCUPIED);

    ros::Duration(1.0).sleep();

    while(ros::ok())
    {
        viz.vizChs(g1);
        ros::Duration(1.0).sleep();
    }
    
    
}
