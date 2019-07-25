#include "maps/prob_map.hpp"
#include "maps/distance_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "strategies/graph_search_strategies.hpp"
#include "path_utils_addons.hpp"
#include "robot/urdf_model.hpp"


using namespace GVP;

void checkNoGpuMemoryLeaks()
{
// Run nvidia-smi and make sure gpu memory does not grow
    std::cout << "Starting trial\n";


    int counter = 0;

    for(int i=0; i<100000; i++)
    {
        PROFILE_START("Copy and Collide");
        DenseGrid g1;
        PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                              Vector3f(2.0,1.0,1.2),
                                                              VOXEL_SIDE_LENGTH/2));
        g1.insertPointCloud(box, PROB_OCCUPIED);

        DenseGrid g2(g1);

        if(g1.collideWith(&g2))
        {
            counter++;
        }
        PROFILE_RECORD("Copy and Collide");

        std::unique_ptr<Belief> bel = std::make_unique<ChsBelief>();
    }

    PROFILE_PRINT_SUMMARY_FOR_SINGLE("Copy and Collide");
    std::cout << "Ran trials and found " << counter << " collision\n";

}


void checkBasicViz(GpuVoxelRvizVisualizer &viz)
{
    DenseGrid g1;
    PointCloud box(geometry_generation::createBoxOfPoints(Vector3f(0.1, 0.5, 1.0),
                                                          Vector3f(1.1, 1.0, 1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box, PROB_OCCUPIED);

    ros::Duration(1.0).sleep();

    while(ros::ok())
    {
        viz.vizGrid(g1, "box", makeColor(1.0, 0, 0, 1.0));
        ros::Duration(1.0).sleep();
    }
}


void viewLinks(GpuVoxelRvizVisualizer &viz)
{
    VictorRightArm v;
    auto link_occupancies = v.getLinkOccupancies();
    while(ros::ok())
    {
        for(DenseGrid &g: link_occupancies)
        {
            viz.vizGrid(g, "link", makeColor(0.0, 1.0, 1.0, 1.0));
            ros::Duration(1.0).sleep();
        }
    }
}


void testAngles(Scenario& scenario, GpuVoxelRvizVisualizer &viz)
{
    VictorKinematics urdf;
    while(true)
    {
        arc_helpers::WaitForInput("Waiting for user input to set position...\n");

        std::ifstream myfile;
        myfile.open("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/config/test_angles.txt");
        std::cout << "file " << (myfile.is_open() ? "is open":"failed to open") << "\n";
        std::vector<double> joint_angles;
        joint_angles.resize(7);
        for(int i=0; i<7; i++)
        {
            myfile >> joint_angles[i];
            std::cout << joint_angles[i] << ", ";
        }
        std::cout << "\n";
        myfile.close();
        VictorRightArmConfig c(joint_angles);
        scenario.getState().robot.set(c.asMap());
        scenario.viz(viz);
        viz.vizEEPosition(c.asVector(), makeColor(0,1,0));
        bool valid = scenario.getState().isPossiblyValid(c);
        std::cout << "EE position: " << urdf.getEEPosition(c.asVector()) << "\n";
        std::cout << "config is " << (valid ? "" : "NOT ") << "possibly valid\n";
    }
}

void testDistanceGrid()
{
    std::cout << "Testing Distance Grid\n";
    DistanceGrid dg;
    DenseGrid g1, g2;
    PointCloud box1(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,1.0),
                                                          Vector3f(2.0,1.0,1.2),
                                                          VOXEL_SIDE_LENGTH/2));
    g1.insertPointCloud(box1, PROB_OCCUPIED);
    dg.mergeOccupied(&g1);
    dg.computeDistances();

    PointCloud box2(geometry_generation::createBoxOfPoints(Vector3f(1.0,0.8,0.5),
                                                           Vector3f(2.0,1.0,0.7),
                                                           VOXEL_SIDE_LENGTH/2));
    g2.insertPointCloud(box2, PROB_OCCUPIED);

    std::cout << "obstacle dist: " << dg.getClosestObstacleDistance(&g2)*VOXEL_SIDE_LENGTH << "\n";
    // std::cout << "dist: " << dg.getObstacleDistance(100, 100, 100) << "\n";

    auto result = dg.getClosestObstacle(&g2);
    std::cout << "pos: " << result.first << ", closest: " << result.second.getObstacle() << "\n";
}


void obstacleBelief(GpuVoxelRvizVisualizer& viz)
{
    BeliefParams bp(BeliefType::CHS);
    TableWithBox scenario(bp, true, true, false);
    std::cout << "num obstacles: " << scenario.unknown_obstacles.obstacles.size() << "\n";
    viz.vizGrid(scenario.unknown_obstacles.occupied, "unknown_base", makeColor(0, 0, 0, 1.0));
    ObstacleBelief bel(scenario.unknown_obstacles, 0.1, std::vector<double>{0,0,0});

    bel.viz(viz);
    DistanceGrid dg;
    viz.vizGrid(scenario.getState().robot.occupied_space, "test_robot", makeColor(0,1,0,1));
    std::cout << "Projecting\n";
    bel.updateCollisionSpace(scenario.getState().robot, 0);
    bel.viz(viz);

    

    // return;
    // ObstacleBelief bel;
    // {
    //     ObstacleConfiguration oc;
    //     Object object;
    //     object.add(AABB(Vector3f(1.7, 1.4, 0.9), Vector3f(2.4, 2.4, .925)));
    //     oc.add(object);
    //     oc.remakeGrid();
    //     bel.addElem(oc, 1.0);
    // }
    // {
    //     ObstacleConfiguration oc;
    //     Object object;
    //     object.add(AABB(Vector3f(1.7, 1.4, 1.0), Vector3f(2.4, 2.4, 1.025)));
    //     oc.add(object);
    //     bel.addElem(oc, 1.0);
    // }
    // {
    //     ObstacleConfiguration oc;
    //     Object object;
    //     object.add(AABB(Vector3f(1.8, 1.4, 1.0), Vector3f(2.5, 2.4, 1.025)));
    //     oc.add(object);
    //     bel.addElem(oc, 1.0);
    // }

    // bel.viz(viz);
}

void StratTest(GpuVoxelRvizVisualizer& viz)
{
    BeliefParams bp(BeliefType::CHS);

    OptimisticGraphSearch strat;

    TableWithBox scenario(bp, true, true, false);

    auto path = strat.applyTo(scenario, viz);

}


int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;
    GpuVoxelRvizVisualizer viz(n);
    ros::Duration(1.0).sleep();



    std::string graph_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph";

    // checkBasicViz(viz);
    // viewLinks(viz);

    // checkNoGpuMemoryLeaks();
    // return 1;

    // testDistanceGrid();
    // return 1;
    // StratTest(viz);

    // obstacleBelief(viz);
    // return 1;


    BeliefParams bp(BeliefType::Obstacle, std::vector<double>{0.0, 0.0, 0.0}, 0.001);

    // GVP::VictorRightArm victor_right;
    // GVP::VictorLeftArmAndBase victor_left;
    // TableWithBox scenario(bp, true, true, true);
    SlottedWall scenario(bp);
    // Bookshelf scenario(true);

    // AStarGraphSearch strat;
    OmniscientGraphSearch strat;
    // OmniscientGraphSearch strat(graph_filepath);
    // OptimisticGraphSearch strat;
    // ParetoCostGraphSearch strat(10.0);
    // UnknownSpaceCostGraphSearch strat(10.0, 0.0001);


    testAngles(scenario, viz);
    return 1;

    
    
    SimulationScenarioTester tester(scenario, n);
    std::cout << "Attempting strategy\n";
    tester.attemptStrategy(strat);
    strat.saveToFile();

    
    PROFILE_PRINT_SUMMARY_FOR_ALL();
    std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
    
}
