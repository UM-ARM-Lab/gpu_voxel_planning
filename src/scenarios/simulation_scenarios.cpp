#include "scenarios/simulation_scenarios.hpp"

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




SimulationScenario::SimulationScenario() : s(victor) {}

void SimulationScenario::initFakeVictor(RosInterface &ri)
{
    //NOTE:!!! These values are duplicated, and harded coded elsewhere in this code
    VictorLeftArmConfig lac(std::vector<double>{1.57, 1.57, 0, 0, 0, 0 ,0});
    ri.setLeftArm(lac);
    ri.setRightGripper(1.5);
}

void SimulationScenario::setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp)
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
    case BeliefType::IID:
        std::cout << "Using IID belief\n";
        s.bel = std::make_unique<IIDBelief>(unknown_obstacles, bp.noise, bp.bias);
        break;
    case BeliefType::Deterministic:
        std::cout << "Using Deterministic\n";
        s.bel = std::make_unique<EmptyBelief>();
        break;
    default:
        std::cout << "Invalid belief type " << bp.belief_type << "\n";
        throw std::invalid_argument("Invalid belief type");
    }

}

void SimulationScenario::validate()
{
    s.robot.set(s.getCurConfig().asMap());
    if(s.robot.occupied_space.overlapsWith(&true_obstacles.occupied))
    {
        std::cerr << "Start configuration overlaps with obstacle\n";
        throw(std::invalid_argument("Start configuration is invalid\n"));
    }
    s.robot.set(goal_config);
    if(s.robot.occupied_space.overlapsWith(&true_obstacles.occupied))
    {
        std::cerr << "Goal configuration overlaps with obstacle\n";
        throw(std::invalid_argument("Goal configuration is invalid\n"));
    }
}


void SimulationScenario::viz(const GpuVoxelRvizVisualizer& viz)
{
    viz.vizGrid(getTrueObstacles(), "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5));
    viz.vizGrid(s.known_obstacles, "known_obstacles", makeColor(0,0,0,1));

    std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
    viz.vizGrid(s.robot_self_collide_obstacles, "passive_robot", robot_color);
    viz.vizGrid(s.robot.occupied_space, "active_robot", robot_color);
    s.bel->viz(viz);
}

void SimulationScenario::addLeftArm()
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

void SimulationScenario::combineObstacles()
{
    for(auto& ob: known_obstacles.obstacles)
    {
        s.known_obstacles.add(&ob.occupied);
    }

    for(const auto& ob: known_obstacles.obstacles)
    {
        true_obstacles.add(ob);
    }
    for(const auto& ob: unknown_obstacles.obstacles)
    {
        true_obstacles.add(ob);
    }
}



/****************************************
 **         Table With Box
 ****************************************/
TableWithBox::TableWithBox(BeliefParams bp, bool table_known, bool visible_cave_known, bool full_cave_known) :
    name(std::string("Table_with_Box_") +
         "table_" + (table_known ? "" : "un") + "known_" + 
         "visible_cave_" + (visible_cave_known ? "" : "un" ) + "known_" + 
         "full_cave_" + (full_cave_known ? "" : "un" ) + "known"
        )
{
    addLeftArm();

    Object table = getTable();
    Object known_cave = getVisibleCave();
    Object cave_back = getCaveBack();

    table_known ? known_obstacles.add(table) : unknown_obstacles.add(table);
    // visible_cave_known ? known_obstacles.add(known_cave) : unknown_obstacles.add(known_cave);
    // full_cave_known ? known_obstacles.add(cave_back) : unknown_obstacles.add(cave_back);

    combineObstacles();


    setPrior(unknown_obstacles, bp);


    // s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
    // s.current_config = VictorRightArmConfig(std::vector<double>{-1.5,1.5,-1.5,-0.4,-1.5,-1.0,1.5}).asMap();
    s.current_config = VictorRightArmConfig(std::vector<double>{-1.0,1.5,-1.5,0.4,-1.5,0.0,1.5}).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();
}

void TableWithBox::setCaveDims()
{
    cavecorner = Vector3f(1.7, 2.0, 0.9);
    caveheight = Vector3f(0.0, 0.0, 0.4);
    cavetopd = Vector3f(0.35, 0.5, 0.033);
    cavesidedim = Vector3f(0.033, cavetopd.y, caveheight.z);
    cavesideoffset = Vector3f(cavetopd.x, 0.0, 0.0);
}

Object TableWithBox::getTable()
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


Object TableWithBox::getVisibleCave()
{
    setCaveDims();
    Object cave;
    cave.add(AABB(cavecorner+caveheight, cavecorner+caveheight+cavetopd+Vector3f(0.033, 0, 0))); //top
    cave.add(AABB(cavecorner, cavecorner+cavesidedim));
    return cave;
}

Object TableWithBox::getCaveBack()
{
    setCaveDims();
    Object cave_back;
    cave_back.add(AABB(cavecorner+cavesideoffset,
                       cavecorner+cavesideoffset+cavesidedim));
    return cave_back;
}




/****************************************
 **         SlottedWall
 ****************************************/
SlottedWall::SlottedWall(BeliefParams bp):
    name("Sloted Wall")
{
    addLeftArm();

    bool all_known = (bp.belief_type == BeliefType::Deterministic);

    known_obstacles.add(getFrontWall());

    if(all_known)
    {
        known_obstacles.add(getSlottedWall());
    }
    else
    {
        unknown_obstacles.add(getSlottedWall());
    }

    combineObstacles();
    setPrior(unknown_obstacles, bp);
    
    // s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
    // goal_config = VictorRightArmConfig(std::vector<double>{0, 0.32, 0, -1.32, -0.2, 0.9, 0.3}).asMap();


    s.current_config = VictorRightArmConfig(std::vector<double>{-1.5, 1.0, -0.5, -0.5,0,0,0}).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>{0, 0.72, -0.3, -1.32, -1.2, 0.9, 0.3}).asMap();

    victor.set(s.current_config);
}

Object SlottedWall::getFrontWall() const
{
    Object front_wall;
    double lower_wall_height = 1.1;
    double gap_height = .4;

    Vector3f lfwc(1.5, 1.6, 0.0); //lower front wall corner
    Vector3f lfwd(0.04, 0.4, lower_wall_height);
    
    Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height); //upper front wall
    Vector3f ufwd(0.04, 0.4, 0.3);

    Vector3f mfwc(1.5, 1.8, 0);  //middle front wall
    Vector3f mfwd(0.04, 0.2, 1.5);

    front_wall.add(AABB(lfwc, lfwc+lfwd));
    front_wall.add(AABB(ufwc, ufwc+ufwd));
    front_wall.add(AABB(mfwc, mfwc+mfwd));
    return front_wall;
}

Object SlottedWall::getSlottedWall() const
{
    Object slotted_wall;
    double lower_wall_height = 1.1;
    double gap_height = .4;

    
   
    Vector3f lswc(1.5, 1.6, 0.0);  // lower side wall corner
    Vector3f lswd(.75, 0.04, lower_wall_height); //lower side wall dims
    Vector3f cswc(1.5, 1.6, lower_wall_height + gap_height); //close side wall corner
    // Vector3f cswd(0.2, 0.04, 0.3);
    Vector3f cswd(0.5, 0.04, 0.3);
    Vector3f fswc(1.95, 1.6, lower_wall_height); //far side wall corner
    Vector3f fswd(0.3, 0.04, 0.6);
    Vector3f mswc(1.95, 1.6, lower_wall_height+gap_height+.1); //
    Vector3f mswd(0.3, 0.04, 0.2);

            
    slotted_wall.add(AABB(lswc, lswc+lswd));  // lower side wall
    slotted_wall.add(AABB(cswc, cswc+cswd));  // close side wall
    slotted_wall.add(AABB(fswc, fswc+fswd));  // far side wall
    slotted_wall.add(AABB(mswc, mswc+mswd));  // middle side wall
    return slotted_wall;
}



/****************************************
 **         Bookshelf
 ****************************************/

Bookshelf::Bookshelf(BeliefParams bp):
    name(std::string("Bookshelf"))
{
    addLeftArm();

    robot::JointValueMap jvm;
    jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
    jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
    jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
    victor.set(jvm);

    Object bookshelf = getBookshelf();
    Object table = getTable();

    known_obstacles.add(bookshelf);

    bool table_known = (bp.belief_type == BeliefType::Deterministic);
    if(table_known)
    {
        known_obstacles.add(table);
    }
    else
    {
        unknown_obstacles.add(table);
    }


    combineObstacles();
    setPrior(unknown_obstacles, bp);
    
    // s.current_config = VictorRightArmConfig(std::vector<double>
    //                                         {-0.9, 1.3, -0.3, -0.8, 0.0, 0.2, 0.3}).asMap();
    s.current_config = VictorRightArmConfig(std::vector<double>
                                            {-1.2, 1.3, -0.8, 0.4, 0.4, 0.3, 0.3}).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>
                                       {0.3, 1.2, -0.3, 1.5, 0, -0.7, -0.9}).asMap();

    victor.set(s.current_config);
}


Object Bookshelf::getBookshelf()
{
    double lower_wall_height = 1.1;
    double gap_height = .4;
    double bookshelf_width = 0.8;
    double bookshelf_height = 1.6;
    double bookshelf_depth = 0.4;
    Vector3f backwallc(1.2, 0.8, 0.0); //backwall cornder
    Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
    Vector3f sidewall(0.04, bookshelf_depth,  bookshelf_height);
    Vector3f swoff(bookshelf_width, 0, 0);
    Vector3f shelf(bookshelf_width + 0.04, bookshelf_depth, 0.02);
    Vector3f shelf_spacing(0, 0, 0.4);

    Vector3f bookc(1.6, 0.82, 1.2);
    Vector3f bookd(0.05, 0.3, 0.3);

    Object bookshelf;

    bookshelf.add(AABB(backwallc, backwallc + backwall_thickness));
    bookshelf.add(AABB(backwallc, backwallc + sidewall));
    bookshelf.add(AABB(backwallc + swoff, backwallc + sidewall + swoff));
    for(int i=0; i<5; i++)
    {
        bookshelf.add(AABB(backwallc + shelf_spacing*(float)i, backwallc + shelf + shelf_spacing*(float)i));
    }
    bookshelf.add(AABB(bookc, bookc+bookd));

    return bookshelf;
}



Object Bookshelf::getTable()
{
    Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
    Vector3f tc(1.7, 1.4, 0.9); //table corner
    Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
    Vector3f tld(.033, 0.033, tc.z); //table leg dims
            
    Object table;
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
 **         CloseWall
 ****************************************/

CloseWall::CloseWall(BeliefParams bp):
    name(std::string("CloseWall"))
{
    addLeftArm();

    robot::JointValueMap jvm;
    jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
    jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
    jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
    victor.set(jvm);

    Object wall = getCloseWall();

    bool wall_known = (bp.belief_type == BeliefType::Deterministic);
    if(wall_known)
    {
        known_obstacles.add(wall);
    }
    else
    {
        unknown_obstacles.add(wall);
    }


    combineObstacles();
    setPrior(unknown_obstacles, bp);
    
    // s.current_config = VictorRightArmConfig(std::vector<double>
    //                                         {-0.9, 1.3, -0.3, -0.8, 0.0, 0.2, 0.3}).asMap();
    s.current_config = VictorRightArmConfig(std::vector<double>
                                            {-0.5, 1.2, -1.5, 0.4, -1.5, 0.0, 1.5}).asMap();
    goal_config = VictorRightArmConfig(std::vector<double>
                                       {-0.0, -0.4, -1.5, -0.4, -1.5, -1.0, 1.5}).asMap();

    victor.set(s.current_config);
}


Object CloseWall::getCloseWall()
{
    double lower_wall_height = 1.1;
    double gap_height = .4;

    double bookshelf_width = 0.8;
    double bookshelf_height = 1.6;
    double bookshelf_depth = 0.4;
    Vector3f backwallc(1.6, 1.2, 0.0); //backwall cornder
    Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
    Vector3f sidewall(0.04, bookshelf_depth,  bookshelf_height);
    Vector3f swoff(bookshelf_width, 0, 0);
    Vector3f shelf(bookshelf_width + 0.04, bookshelf_depth, 0.02);
    Vector3f shelf_spacing(0, 0, 0.4);

    Object wall;

    wall.add(AABB(backwallc, backwallc + backwall_thickness));
    wall.add(AABB(backwallc, backwallc + sidewall));
    wall.add(AABB(backwallc + swoff, backwallc + sidewall + swoff));
    for(int i=0; i<5; i++)
    {
        wall.add(AABB(backwallc + shelf_spacing*(float)i, backwallc + shelf + shelf_spacing*(float)i));
    }

    return wall;
}



