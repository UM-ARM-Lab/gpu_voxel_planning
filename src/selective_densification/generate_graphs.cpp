#include "strategies/victor_selective_densification.hpp"
#include "sd_params.hpp"
#include <ros/ros.h>


using namespace GVP;

const std::string basepath_sd = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD/";
const std::string basepath_id = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/ID/";



void makeSDGraph(int seed)
{
    // std::string filepath = basepath + "seed" + std::to_string(seed) + "/";
    std::string filename = "seed" + std::to_string(seed) + ".graph";
    std::string filepath = basepath_sd + filename;
    SDRoadmap sd_graph(filepath, seed);
}



void makeIDGraph(int seed)
{
    std::string filename = "seed" + std::to_string(seed) + ".graph";
    std::string filepath = basepath_id + filename;
    IDRoadmap id_graph(filepath, seed);
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "selective_densification_graph_generation");
    ros::NodeHandle n;

    ros::Duration(1.0).sleep();

    for(int seed=0; seed<10; seed++)
    {
        // makeSDGraph(seed);
        makeIDGraph(seed);
    }


}
