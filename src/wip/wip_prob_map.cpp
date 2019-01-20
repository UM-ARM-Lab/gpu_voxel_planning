#include "prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "gpu_voxel_rviz_visualization.hpp"
#include "robot_model.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "path_utils_addons.hpp"

using namespace GVP;

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


void checkBasicViz(GpuVoxelRvizVisualizer &viz)
{
    ProbGrid g1;
    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(0.1, 0.5, 1.0),
                                                          Vector3f(1.1, 1.0, 1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box, PROB_OCCUPIED);

    ros::Duration(1.0).sleep();

    while(ros::ok())
    {
        viz.vizChs(g1);
        ros::Duration(1.0).sleep();
    }
}


void testAngles(Scenario& scenario, GpuVoxelRvizVisualizer &viz)
{
    
    while(true)
    {
        arc_helpers::WaitForInput("Waiting for user input to set position...\n");

        std::ifstream myfile;
        myfile.open("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/config/test_angles.txt");
        std::cout << "file is open: " << myfile.is_open() << "\n";
        std::vector<double> goal_tmp;
        goal_tmp.resize(7);
        for(int i=0; i<7; i++)
        {
            myfile >> goal_tmp[i];
            std::cout << goal_tmp[i] << ", ";
        }
        std::cout << "\n";
        myfile.close();
        VictorRightArmConfig c(goal_tmp);
        scenario.getState().robot.set(c.asMap());
        viz.vizScenario(scenario);
        bool valid = scenario.getState().isPossiblyValid(c);
        std::cout << "config is " << (valid ? "" : "NOT ") << "possibly valid\n";
    }
}


int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;
    GpuVoxelRvizVisualizer viz(n);

    // checkBasicViz(viz);

    ros::Duration(1.0).sleep();
    // GVP::VictorRightArm victor_right;
    // GVP::VictorLeftArmAndBase victor_left;
    TableWithBox scenario;
    ScenarioTester tester(scenario, n);

    double i = 0;

    // testAngles(scenario, viz);

    
    Path p = interpolate(VictorRightArmConfig(scenario.getState().current_config),
                         VictorRightArmConfig(scenario.goal_config), 0.02);
    std::cout << "path has " << p.size() << " points\n";
    tester.attemptPath(p);
    
    // while(ros::ok())
    // {
    //     // victor_left.set(VictorLeftArmConfig(std::vector<double>{i, i, i, i, i, i, i}).asMap());
    //     i+= 0.1;
        
    //     viz.vizScenario(scenario);
    //     ros::Duration(0.1).sleep();
    // }
    
    
}
