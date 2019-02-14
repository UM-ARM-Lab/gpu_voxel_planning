#include "strategies/ompl_strategies.hpp"
#include "hacky_functions.hpp"


using namespace GVP;
namespace ob = ompl::base;
namespace og = ompl::geometric;



std::shared_ptr<ob::RealVectorStateSpace> OMPL_Strategy::makeSpace()
{
    auto space = std::make_shared<ob::RealVectorStateSpace>();
    for(int i=0; i<7; i++)
    {
        space->addDimension(right_joint_lower_deg[i]*torad, right_joint_upper_deg[i]*torad);
    }

}

Path OMPL_Strategy::applyTo(Scenario &scenario)
{
    auto state_validity_fn = [&](const ob::State *ompl_state)
        {
            return isOmplStateValid(ompl_state, scenario.getState());
        };


    auto space = makeSpace();
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(state_validity_fn);


    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    
    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(60);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);

    }
}

bool OMPL_Strategy::isOmplStateValid(const ompl::base::State *ompl_state,
                                     const GVP::State &gvp_state)
{
    VictorRightArmConfig config(ompl_state->as<ob::RealVectorStateSpace::StateType>()->values);
    gvp_state.robot.set(config.asMap());
    return !gvp_state.robot.occupied_space.overlapsWith(&gvp_state.known_obstacles);
}



std::string RRT_Strategy::getName() const
{
    return "RRT Strategy";
}
