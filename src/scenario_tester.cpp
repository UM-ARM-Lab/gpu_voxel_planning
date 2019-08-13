#include "scenario_tester.hpp"
#include "gvp_exceptions.hpp"

using namespace GVP;


/*******************************
 *  Simulation Scenario Tester
*******************************/
bool SimulationScenarioTester::attemptPath(const std::vector<VictorRightArmConfig> &path)
{
    num_path_attempts++;
    for(const auto &c:path)
    {
        if(!scenario.getSimulationState().move(c, scenario.getTrueObstacles(), ri))
        {
            PROFILE_START("Viz_scenario");
            scenario.viz(ri.viz);
            PROFILE_RECORD("Viz_scenario");
            PROFILE_RECORD_DOUBLE("Bump", 0);
            ri.viz.vizEEPath(path, "invalid_attempt", num_path_attempts, makeColor(1.0, 0.0, 0.0));
            last_invalid = true;
            return false;
        }
        PROFILE_START("Viz_scenario");
        //Commented out for speed!
        scenario.viz(ri.viz);
        
        PROFILE_RECORD("Viz_scenario");
        // ros::Duration(0.01).sleep();
        ros::Duration(0.001).sleep();
    }
    if(!last_invalid)
    {
        ri.viz.vizEEPath(path, "valid_attempt", num_path_attempts, makeColor(0.0, 0.0, 1.0));
    }
    last_invalid = false;
    return true;
}

bool SimulationScenarioTester::attemptStrategy(Strategy &strategy)
{
    PROFILE_RECORD_DOUBLE("Scenario: " + scenario.getName(), 0);
    PROFILE_RECORD_DOUBLE("Strategy: " + strategy.getName(), 0);
    PROFILE_RECORD_DOUBLE("Belief: " + scenario.belief_name, 0);

    std::cout << "Testing simulation scenario: "
              << scenario.getName() << ", "
              << strategy.getName() << ", "
              << scenario.belief_name
              << "\n";
    ros::Duration(1.0).sleep();
    
    scenario.viz(ri.viz);
    scenario.initFakeVictor(ri);
    try{
        scenario.validate();
    }
    catch(std::invalid_argument &e)
    {
        scenario.viz(ri.viz);
        throw e;
    }
    
    std::string name = getName(strategy);
    while(!scenario.completed())
    {
        // arc_helpers::WaitForInput();
        PROFILE_START(name + " Planning Time");
        std::vector<VictorRightArmConfig> path;
        try
        {
            path = strategy.applyTo(scenario, ri.viz);
        }
        catch(SearchError &e)
        {
            std::cout << "No path found: " << e.what() << "\n";
            PROFILE_RECORD_DOUBLE("Failed", 0);
            return false;
        }
        PROFILE_RECORD(name + " Planning Time");

        ri.viz.vizEEPath(path, "attempt", 0, makeColor(0.0, 0.0, 0.0));
        // std::cout << "path found with " << path.size() << " verts\n";
        
        PROFILE_START(name + " Motion Time");
        attemptPath(path);
        PROFILE_RECORD(name + " Motion Time");

    }
    std::cout << "Reached Goal with cost " << scenario.getState().accumulated_cost << "\n";
    PROFILE_RECORD_DOUBLE("accumulated_cost", scenario.getState().accumulated_cost);
    return true;
}

std::string SimulationScenarioTester::getName(const Strategy &strategy) const
{
    return scenario.getName() + ", " +  strategy.getName();
}





/*******************************
 *    Real Scenario Tester
*******************************/
bool RealScenarioTester::attemptPath(const std::vector<VictorRightArmConfig> &path)
{
    num_path_attempts++;
    bool succeeded = scenario.getRealState().move(path, ri);

    PROFILE_START("Viz_scenario");
    scenario.viz(ri.viz);
    PROFILE_RECORD("Viz_scenario");
    //     // ros::Duration(0.01).sleep();
    //     ros::Duration(0.001).sleep();
    // }
    // return true;


    if(!last_invalid)
    {
        if(succeeded)
        {
            ri.viz.vizEEPath(path, "valid_attempt", num_path_attempts, makeColor(0.0, 0.0, 1.0));
        }
        else
        {
            ri.viz.vizEEPath(path, "invalid_attempt", num_path_attempts, makeColor(1.0, 0.0, 0.0));
        }
    }
    last_invalid = !succeeded;

    
    return succeeded;
}

bool RealScenarioTester::attemptStrategy(Strategy &strategy)
{
    PROFILE_RECORD_DOUBLE("Scenario: " + scenario.getName(), 0);
    PROFILE_RECORD_DOUBLE("Strategy: " + strategy.getName(), 0);
    PROFILE_RECORD_DOUBLE("Belief: " + scenario.belief_name, 0);

    std::cout << "Attempt Strategy\n";
    std::cout << "Viz Scenario\n";
    scenario.viz(ri.viz);
    std::cout << "Init Fake Victor\n";
    scenario.initFakeVictor(ri);
    try{
        std::cout << "Validate\n";
        scenario.validate();
    }
    catch(std::invalid_argument &e)
    {
        scenario.viz(ri.viz);
        throw e;
    }

    std::cout << "getname\n";
    std::string name = getName(strategy);
    while(!scenario.completed())
    {
        PROFILE_START(name + " Planning Time");
        std::vector<VictorRightArmConfig> path;
        try
        {
            std::cout << "plan\n";
            path = strategy.applyTo(scenario, ri.viz);
        }
        catch(std::logic_error &e)
        {
            std::cout << "No path found\n";
            return false;
        }
        PROFILE_RECORD(name + " Planning Time");

        ri.viz.vizEEPath(path, "Path Found", 0, makeColor(0.0, 0.0, 1.0));
        // std::cout << "path found with " << path.size() << " verts\n";
        
        PROFILE_START(name + " Motion Time");
        if(attemptPath(path))
        {
            path_taken.insert(path_taken.end(), path.begin(), path.end());
        }
        PROFILE_RECORD(name + " Motion Time");

    }
    std::cout << "Reached Goal with cost " << scenario.getState().accumulated_cost << "\n";
    PROFILE_RECORD_DOUBLE("accumulated_cost", scenario.getState().accumulated_cost);
    return true;
}

bool RealScenarioTester::reversePath()
{
    std::reverse(path_taken.begin(), path_taken.end());
    scenario.getRealState().move(path_taken, ri);
}

std::string RealScenarioTester::getName(const Strategy &strategy) const
{
    return scenario.getName() + ", " +  strategy.getName();
}
