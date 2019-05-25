#include "scenarios/real_scenario.hpp"

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

// void RealScenario::setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp)
void RealScenario::setPrior(BeliefParams bp)
{
    switch(bp.belief_type)
    {
    case BeliefType::CHS:
        std::cout << "Using CHS belief\n";
        s.bel = std::make_unique<ChsBelief>();
        break;
    // case BeliefType::Obstacle:
    //     std::cout << "Using Obstacle belief\n";
    //     s.bel = std::make_unique<ObstacleBelief>(unknown_obstacles, bp.noise, bp.bias);
    //     break;
    case BeliefType::Bonkers:
        std::cout << "Using Bonkers belief\n";
        s.bel = std::make_unique<ObstacleBelief>(getBonkersBelief(), bp.noise, bp.bias);
        break;
    // case BeliefType::MoEObstacle:
    //     std::cout << "Using MoE Obstalcebelief\n";
    //     s.bel = std::make_unique<MoEBelief>(unknown_obstacles, bp.noise, bp.bias);
    //     break;
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
    s.robot.set(goal_config);
    if(s.robot.occupied_space.overlapsWith(&known_obstacles.occupied))
    {
        std::cerr << "Goal configuration overlaps with obstacle\n";
        throw(std::invalid_argument("Goal configuration is invalid\n"));
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




/****************************************
 **         Real Empty
 ***************************************/
RealEmpty::RealEmpty(BeliefParams bp) :
    name("RealEmpty")
{
    addLeftArm();

    // known_obstacles.add(getTable());

    // setPrior(unknown_obstacles, bp);
    setPrior(bp);

    for(auto& ob: known_obstacles.obstacles)
    {
        s.known_obstacles.add(&ob.occupied);
    }
    s.current_config = VictorRightArmConfig(std::vector<double>{
            -1.231, 1.225, -0.666, -0.893, -1.496, 0.804, -0.037
                }).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>{
            0.274, 0.712, -0.502, -1.131, -1.339, 1.346, -0.03
                }).asMap();
}



/****************************************
 **         RealTable
 ****************************************/
RealTable::RealTable(BeliefParams bp) :
    name("RealTable")
{
    addLeftArm();

    known_obstacles.add(getTable());

    // setPrior(unknown_obstacles, bp);
    setPrior(bp);

    for(auto& ob: known_obstacles.obstacles)
    {
        s.known_obstacles.add(&ob.occupied);
    }

    s.current_config = VictorRightArmConfig(std::vector<double>{
            0,0,0,0,0,0,0
                }).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();

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




