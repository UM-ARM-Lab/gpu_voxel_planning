#include "strategies/ompl_strategies.hpp"
#include "ompl_utils.hpp"
#include "hacky_functions.hpp"
#include "path_utils_addons.hpp"

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
    return space;
}

Path OMPL_Strategy::applyTo(Scenario &scenario)
{
    auto state_validity_fn = [&](const ob::State *ompl_state)
        {
            return isOmplStateValid(ompl_state, scenario.getState());
        };


    auto space = makeSpace();

    space->setLongestValidSegmentFraction(discretization/space->getMaximumExtent());
    
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(state_validity_fn);

    ss.setPlanner(makePlanner(ss.getSpaceInformation()));
    ob::ScopedState<> start =
        ompl_utils::toScopedState(scenario.getState().getCurConfig().asVector(),
                                  space);
    ob::ScopedState<> goal =
        ompl_utils::toScopedState(VictorRightArmConfig(scenario.goal_config).asVector(),
                                  space);
    
    ss.setStartAndGoalStates(start, goal);

    PROFILE_START("PathLength");
    PROFILE_START("OMPL Planning");
    ob::PlannerStatus solved = ss.solve(60);
    PROFILE_RECORD("OMPL Planning");
    std::cout << "Longest valid segment: " << space->getLongestValidSegmentLength() << "\n";
    if (solved)
    {
        PathUtils::Path pu_path = ompl_utils::omplPathToDoublePath(&ss.getSolutionPath(),
                                                                   ss.getSpaceInformation());
        return smooth(GVP::toPath(pu_path), scenario.getState());
    }

    throw std::runtime_error("Path not found");
}

bool OMPL_Strategy::isOmplStateValid(const ompl::base::State *ompl_state,
                                     GVP::State &gvp_state)
{
    PROFILE_START("OMPL Configuration Check");
    VictorRightArmConfig config(ompl_state->as<ob::RealVectorStateSpace::StateType>()->values);
    // gvp_state.robot.set(config.asMap());
    // bool valid = !gvp_state.robot.occupied_space.overlapsWith(&gvp_state.known_obstacles);
    bool valid = gvp_state.isPossiblyValid(config);
    PROFILE_RECORD("OMPL Configuration Check");
    return valid;
}



/************************
 *  RRT
 ***********************/
ompl::base::PlannerPtr RRT_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    return std::make_shared<og::RRTConnect>(si);
}

std::string RRT_Strategy::getName() const
{
    return "RRT_Strategy";
}

Path RRT_Strategy::smooth(Path gvp_path, State &state)
{
    // print the path to screen
    // PROFILE_START("OMPL Smoothing");
    // ss.simplifySolution();
    // PROFILE_RECORD("OMPL Smoothing");
    // ss.getSolutionPath().print(std::cout);
    // PROFILE_RECORD_DOUBLE("OMPL path length", PathUtils::length(pu_path));
    // std::cout << "Path length: " << PathUtils::length(pu_path) << "\n";

    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    for(int i=0; i<30; i++)
    {
        gvp_path = GVP::smooth(gvp_path, state, discretization);
        PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    }

    return GVP::densify(gvp_path, discretization);
}





/************************
 *  BITStar
 ***********************/
ompl::base::PlannerPtr BIT_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    return std::make_shared<og::BITstar>(si);
}

std::string BIT_Strategy::getName() const
{
    return "BIT_Strategy";
}

Path BIT_Strategy::smooth(Path gvp_path, State &state)
{
    return GVP::densify(gvp_path, discretization);
}



