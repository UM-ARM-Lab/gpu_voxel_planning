#include "gpu_voxel_planning/scenarios/real_scenario.hpp"
#undef likely  //boost::likely conflict with arc_helpers
#include <arm_pointcloud_utilities/load_save_to_file.h>


using namespace GVP;

static ObstacleConfiguration getBonkersBelief()
{
    ObstacleConfiguration oc;
    Object chair;

    chair.add(AABB(Vector3f(1.0, 1.0, 0.0), Vector3f(1.05, 1.05, 1.2)));
    chair.add(AABB(Vector3f(1.5, 1.0, 0.0), Vector3f(1.55, 1.05, 1.2)));
    chair.add(AABB(Vector3f(1.0, 1.5, 0.0), Vector3f(1.05, 1.55, 0.6)));
    chair.add(AABB(Vector3f(1.5, 1.5, 0.0), Vector3f(1.55, 1.55, 0.6)));
    chair.add(AABB(Vector3f(1.0, 1.0, 0.55), Vector3f(1.55, 1.55, 0.6)));

    chair.add(AABB(Vector3f(1.0, 1.0, 1.1), Vector3f(1.5, 1.05, 1.2)));

    chair.shift(Vector3f(0.0, 2.0, 0));
    
    oc.add(chair);
    return oc;
}




RealScenario::RealScenario() : s(victor) {}

void RealScenario::initFakeVictor(RosInterface &ri)
{
    //NOTE:!!! These values are duplicated, and harded coded elsewhere in this code
    // VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0 ,0});
    // ri.setLeftArm(lac);
    // ri.setRightGripper(1.5);
}

void RealScenario::setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp)
// void RealScenario::setPrior(BeliefParams bp)
{
    belief_name = bp.toString();
    switch(bp.belief_type)
    {
    case BeliefType::CHS:
        std::cout << "Using CHS belief\n";
        s.bel = std::make_unique<ChsBelief>();
        break;
    case BeliefType::Obstacle:
        std::cout << "Using Obstacle belief\n";
        s.bel = std::make_unique<ObstacleBelief>(unknown_obstacles, bp.noise, bp.bias);
        break;
    case BeliefType::Bonkers:
        std::cout << "Using Bonkers belief\n";
        s.bel = std::make_unique<ObstacleBelief>(getBonkersBelief(), bp.noise, bp.bias);
        break;
    case BeliefType::MoEObstacle:
        std::cout << "Using MoE Obstalcebelief\n";
        s.bel = std::make_unique<MoEBelief>(unknown_obstacles, bp.noise, bp.bias);
        break;
    case BeliefType::MoEBonkers:
        std::cout << "Using MoE Obstalcebelief\n";
        s.bel = std::make_unique<MoEBelief>(getBonkersBelief(), bp.noise, bp.bias);
        break;
    // case BeliefType::IID:
    //     std::cout << "Using IID belief\n";
    //     s.bel = std::make_unique<IIDBelief>(unknown_obstacles, bp.noise, bp.bias);
    //     break;
    default:
        std::cout << "Invalid belief type " << bp.belief_type << "\n";
        throw std::invalid_argument("Invalid belief type");
    }        
}

void RealScenario::validate()
{
    s.robot.set(s.getCurConfig().asMap());
    if(s.robot.occupied_space.overlapsWith(&known_obstacles.occupied))
    {
        std::cerr << "Start configuration overlaps with obstacle\n";
        throw(std::invalid_argument("Start configuration is invalid\n"));
    }

    if(known_goal_config.has_value()) {
        auto goals = getPossibleGoals();
        assert(goals.size() == 1);
        s.robot.set(goals.at(0));
//        s.robot.set(goal_config);
        if (s.robot.occupied_space.overlapsWith(&known_obstacles.occupied)) {
            std::cerr << "Goal configuration overlaps with obstacle\n";
            throw (std::invalid_argument("Goal configuration is invalid\n"));
        }
    }
}


void RealScenario::viz(const GpuVoxelRvizVisualizer& viz)
{
    // viz.vizGrid(getTrueObstacles(), "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5));
    viz.vizGrid(s.known_obstacles, "known_obstacles", makeColor(0,0,0,1));

    std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
    viz.vizGrid(s.robot_self_collide_obstacles, "passive_robot", robot_color);
    viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);
    
    viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);
    s.bel->viz(viz);
}

void RealScenario::addLeftArm()
{
    VictorLeftArmAndBase left;
    VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0 ,0});
    left.set(lac.asMap());
    s.robot_self_collide_obstacles.add(&left.occupied_space);

    robot::JointValueMap jvm;
    jvm["victor_right_gripper_fingerA_joint_2"] = 1.5;
    jvm["victor_right_gripper_fingerB_joint_2"] = 1.5;
    jvm["victor_right_gripper_fingerC_joint_2"] = 1.5;
    victor.set(jvm);
}

DenseGrid RealScenario::loadPointCloudFromFile()
{
    const std::pair<std::string, Eigen::Matrix3Xf> deserialized =
        arm_pointcloud_utilities::LoadPointsetFromFile(ros::package::getPath("arm_pointcloud_utilities") + "/logs/point_cloud_latest.compressed");

    const Eigen::Matrix3Xf &mat = deserialized.second;
    std::cout << "Loading matrix of size " << mat.rows() << ", " << mat.cols() << "\n";
    std::vector<Vector3f> points;
    for(size_t i=0; i<mat.cols(); i++)
    {
        if(std::isnan(mat(0,i)) || std::isnan(mat(1,i)) || std::isnan(mat(2,i)))
        {
            continue;
        }
        // std::cout << mat(0, i) << ", " << mat(1,i) << ", " << mat(2,i) << "\n";
        Vector3f p(1.0+mat(0,i), 2.0+mat(1,i), mat(2,i));

        //right arm
        // if(p.x < 1.7 && p.y < 2.0 && p.z > 1.0)
        //     continue;

        // if(p.z > 0.5 && p.z < 1.05 &&
        //    p.y > 1.6)
        // {
        //     continue;
        // }

        if(p.y > 2.3)
        {
            continue;
        }



        //no point cloud
        // continue;
        
        points.push_back(p);
    }
    DenseGrid g;
    g.insertPointCloud(points, PROB_OCCUPIED);
    return g;
}





/****************************************
 **         Real Empty
 ***************************************/
RealEmpty::RealEmpty(BeliefParams bp) :
    name("Fridge")
{
    addLeftArm();

    unknown_obstacles.add(getTable());
    

    setPrior(unknown_obstacles, bp);
     // setPrior(bp);

    

    DenseGrid kinect = loadPointCloudFromFile();
    // s.known_obstacles.add(&kinect);

    for(auto& ob: known_obstacles.obstacles)
    {
        s.known_obstacles.add(&ob.occupied);
    }
    //TODO: Read this in from a config file
    s.current_config = VictorRightArmConfig(std::vector<double>{
            1.955, -1.141, 0.644, 0.949, -2.459, 0.0, -2.107
            // 1.662, -0.876, 0.453, 1.474, -2.433, 0.196, -0.62
            // -1.078, -0.907, 2.411, -1.024, 0.558, 1.197, 0.929
            // -1.309, 0.834, 0.211, -0.21, 0.275, 0.516, -1.929
            // -0.859, 0.357, 0.578, -0.226, 0.197, 0.492, -1.502
            // -0.71, 0.376, 0.261, -0.195, -0.524, 0.493, -0.425
            // 0.053, 0.058, 0.633, -0.504, -0.595, 0.684, -0.357
            // -1.453, 0.445, 1.602, -0.476, -0.867, 0.316, -0.646
            // -1.701, 1.231, 1.383, -0.271, -0.957, 0.284, -0.765
            // -1.513, 1.217, 0.954, -0.935, -1.949, 0.879, -0.029
            // -1.231, 1.225, -0.666, -0.893, -1.496, 0.804, -0.037
                }).asMap();
    auto goal_config = VictorRightArmConfig(std::vector<double>{
            2.212, -0.783, 1.117, 0.77, -2.426, 0.327, -2.181
            // 2.105, -0.761, 0.902, 1.569, -1.7, 0.142, 0.315
            // -0.424, 0.82, 0.338, -0.946, 0.173, 0.683, -1.498
            // -0.873, 0.771, 0.449, -0.781, 0.102, 0.936, -1.51
            // -1.286, 1.025, 1.334, -1.051, -0.856, 0.946, -1.026
            // 0.274, 0.712, -0.502, -1.131, -1.339, 1.346, -0.03
                }).asMap();
    setKnownGoalConfig(goal_config);
}

Object RealEmpty::getTable()
{
    // Object table; //table is actually the frige wall now
    // // Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    // // Vector3f tc(1.7, 1.4, 0.9); //table corner
    // // Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
    // // Vector3f tld(.033, 0.033, tc.z); //table leg dims
    // Vector3f td(2.0 * 0.0254, 12.0 * 0.0254, 24.0 * 0.0254); //table dimensions
    // Vector3f tc(2.5, 2.0, 0.9); //table corner
    // Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
    // Vector3f tld(.033, 0.033, tc.z); //table leg dims


    // table.add(AABB(tc, tc+td));
    // // table.add(AABB(tcf, tcf+tld));
    // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
    // //                Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
    // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
    // //                Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
    // // table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
    // //                Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
    // return table;

    Object table;
    Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    Vector3f tc(1.7, 1.4, 0.9); //table corner
    Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
    Vector3f tld(.033, 0.033, tc.z); //table leg dims


    table.add(AABB(tc, tc+td));
    table.add(AABB(tcf, tcf+tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
    return table;

}




/****************************************
 **         RealTable
 ****************************************/
RealTable::RealTable(BeliefParams bp) :
    name("RealTable")
{
    addLeftArm();

    // known_obstacles.add(getTable());

    setPrior(unknown_obstacles, bp);
    // setPrior(bp);

    for(auto& ob: known_obstacles.obstacles)
    {
        // s.known_obstacles.add(&ob.occupied);
    }

    //TODO: Read these in from a config file
    s.current_config = VictorRightArmConfig(std::vector<double>{
            1.955, -1.141, 0.644, 0.949, -2.459, 0.0, -2.107
            // 0,0,0,0,0,0,0
                }).asMap();
    auto goal_config = VictorRightArmConfig(std::vector<double>{
                        2.212, -0.783, 1.117, 0.77, -2.426, 0.327, -2.181
                            // -0.15, 1.0, 0, -0.5, 0, 1.0, 0
                            }).asMap();
    setKnownGoalConfig(goal_config);

}

Object RealTable::getTable()
{
    Object table;
    Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    Vector3f tc(1.7, 1.4, 0.9); //table corner
    Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
    Vector3f tld(.033, 0.033, tc.z); //table leg dims


    table.add(AABB(tc, tc+td));
    table.add(AABB(tcf, tcf+tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
    table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                   Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
    return table;
}




