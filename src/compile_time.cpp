#include <ros/ros.h>

#include "maps/prob_map.hpp"
#include "common_names.hpp"

// #include <graph_planner/halton_graph.hpp>
// #include <arc_utilities/eigen_helpers.hpp>


//Removing victor_selective_densification
// #include "gpu_voxel_rviz_visualization.hpp"  //short 6s
// #include "beliefs/beliefs.hpp" //short 6s
// #include "state.hpp" //short 6s
// #include "scenarios.hpp" //short 6s
#include "scenario_tester.hpp"  //long 35s
// #include "strategies/strategies.hpp"


// #include "strategies/graph_search_strategies.hpp"
// #include "obstacles/obstacles.hpp" // short ~5s
// #include <arc_utilities/arc_helpers.hpp> //long ~32s
// #include <graph_planner/dijkstras_addons.hpp> //long ~32s
// #include <graph_planner/halton_graph.hpp> //long ~34s
// #include <graph_planner/increasing_density_halton.hpp> //long ~34s
// #include "strategies/victor_selective_densification.hpp" //long ~34s
// #include "gpu_voxel_rviz_visualization.hpp"  //long ~37s
// #include "beliefs/beliefs.hpp" // long ~30s
// #include "scenario_tester.hpp" //long ~30s

// using namespace GVP;

void tmp()
{
    DenseGrid g1;
    PointCloud box1(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                          Vector3f(2.0,1.0,1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box1, PROB_OCCUPIED);
    
}

// void obstacleBelief(GpuVoxelRvizVisualizer& viz)
// {
//     GVP::TableWithBox scenario(true, true, false);
//     std::cout << "num obstacles: " << scenario.unknown_obstacles.obstacles.size() << "\n";
//     GVP::ObstacleBelief bel(scenario.unknown_obstacles, 0.1, std::vector<double>{0,0,0});

//     bel.viz(viz);
// }

int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;
    // GpuVoxelRvizVisualizer viz(n);
    ros::Duration(1.0).sleep();

    tmp();
}

