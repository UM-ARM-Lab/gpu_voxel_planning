#include "strategies/victor_selective_densification.hpp"
#include "sd_params.hpp"
#include <ros/ros.h>


using namespace GVP;

const std::string basepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD/";



void makeGraph(int seed)
{
    // std::string filepath = basepath + "seed" + std::to_string(seed) + "/";
    std::string filename = "seed" + std::to_string(seed) + ".graph";
    std::string filepath = basepath + filename;
    SDRoadmap sd_graph(filepath, seed);
}






int main(int argc, char* argv[])
{
    ros::init(argc, argv, "selective_densification_graph_generation");
    ros::NodeHandle n;

    ros::Duration(1.0).sleep();

    for(int seed=0; seed<10; seed++)
    {
        makeGraph(seed);
    }


}
