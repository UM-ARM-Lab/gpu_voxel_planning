#include "gpu_voxel_planning/strategies/ompl_strategies.hpp"
#include "gpu_voxel_planning/ompl_utils.hpp"
#include "gpu_voxel_planning/hacky_functions.hpp"
#include "gpu_voxel_planning/path_utils_addons.hpp"
#include "gpu_voxel_planning/ompl_modifications/custom_bitstar.h"
#include "gpu_voxel_planning/gvp_exceptions.hpp"
#include "gpu_voxel_planning/sd_params.hpp"

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

Path OMPL_Strategy::applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz_)
{
    viz = &viz_;
    
    auto state_validity_fn = [&](const ob::State *ompl_state)
        {
            return isOmplStateValid(ompl_state, scenario.getState());
        };


    auto space = makeSpace();

    // std::cout << "max extent: " << space->getMaximumExtent() << "\n";

    //Need to divide by 2, due to the way the edge checking works.
    //Any segment shorter than the LongestSegment length is not checked
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
    ob::PlannerStatus solved = ss.solve(PLANNER_TIMEOUT);
    PROFILE_RECORD("OMPL Planning");
    std::cout << "Longest valid segment: " << space->getLongestValidSegmentLength() << "\n";
    if (!solved)
    {
        PROFILE_RECORD_DOUBLE("PathLength", std::numeric_limits<double>::max());
        throw SearchError("Path not found");
    }
    
    PROFILE_RECORD_DOUBLE("SetRobotConfig before smoothing",
                           arc_utilities::Profiler::getData("Set robot config").size());

    PathUtils::Path pu_path = ompl_utils::omplPathToDoublePath(&ss.getSolutionPath(),
                                                                   ss.getSpaceInformation());
    return smooth(GVP::toPath(pu_path), scenario.getState());
}

void OMPL_Strategy::vizPoint(VictorRightArmConfig& config, bool valid)
{
    if(valid)
    {
        viz->vizEEPosition(config.asVector(), makeColor(0,1,1), viz_id++);
    }
    else
    {
        viz->vizEEPosition(config.asVector(), makeColor(1,0,0), viz_id++);
    }
}

bool OMPL_Strategy::isOmplStateValid(const ompl::base::State *ompl_state,
                                     GVP::State &gvp_state)
{
    PROFILE_START("OMPL Configuration Check");
    VictorRightArmConfig config(ompl_state->as<ob::RealVectorStateSpace::StateType>()->values);
    // gvp_state.robot.set(config.asMap());
    // bool valid = !gvp_state.robot.occupied_space.overlapsWith(&gvp_state.known_obstacles);

    // double d = EigenHelpers::Distance(config.asVector(), previously_sampled_point);
    // std::cout << "Checking config: " << PrettyPrint::PrettyPrint(config.asVector()) << "\n";

    previously_sampled_point = config.asVector();
    bool valid = gvp_state.isPossiblyValid(config);
    PROFILE_RECORD("OMPL Configuration Check");

    if(VISUALIZE)
    {
        vizPoint(config, valid);
    }

    // std::cout << "Distance between samples: " << d << "";
    // if(!valid)
    // {
    //     std::cout << "  invalid\n";
    // }
    // else
    // {
    //     std::cout << "\n";
    // }
    
    
    return valid;
}



/************************
 *  RRT
 ***********************/
ompl::base::PlannerPtr RRT_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    std::shared_ptr<og::RRTConnect> pp = std::make_shared<og::RRTConnect>(si);
    pp->setRange(discretization*50);
    std::cout << "Range: " << pp->getRange() << "\n";
    return pp;
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

    std::mt19937 rng;
    rng.seed(42);
    
    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            gvp_path = GVP::smooth(gvp_path, state, discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
        }
    }

    return GVP::densify(gvp_path, discretization);
}





/************************
 *  BITStar
 ***********************/
ompl::base::PlannerPtr BIT_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    // return std::make_shared<og::custom_BITstar>(si);
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



/************************
 *  SPARS
 ***********************/
ompl::base::PlannerPtr SPARS_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    // return std::make_shared<og::custom_BITstar>(si);
    auto spars = std::make_shared<og::SPARS>(si);
    spars->setMaxFailures(100);
    spars->setDenseDeltaFraction(0.00005);
    spars->setSparseDeltaFraction(0.01);
    spars->setStretchFactor(1.5);
    return spars;
}

std::string SPARS_Strategy::getName() const
{
    return "SPARS_Strategy";
}

Path SPARS_Strategy::smooth(Path gvp_path, State &state)
{
    return GVP::densify(gvp_path, discretization);
}



// /************************
//  *  SPARSTWO
//  ***********************/
// ompl::base::PlannerPtr SPARSTWO_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
// {
//     // return std::make_shared<og::custom_BITstar>(si);
//     return std::make_shared<og::SPARStwo>(si);
// }

// std::string SPARSTWO_Strategy::getName() const
// {
//     return "SPARSTWO_Strategy";
// }

// Path SPARSTWO_Strategy::smooth(Path gvp_path, State &state)
// {
//     return GVP::densify(gvp_path, discretization);
// }




/************************
 *  SST
 ***********************/
ompl::base::PlannerPtr SST_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    std::shared_ptr<og::SST> pp = std::make_shared<og::SST>(si);
    pp->setRange(discretization*50);
    std::cout << "Range: " << pp->getRange() << "\n";
    return pp;
}

std::string SST_Strategy::getName() const
{
    return "SST_Strategy";
}

Path SST_Strategy::smooth(Path gvp_path, State &state)
{
    std::mt19937 rng;
    rng.seed(42);
    
    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            gvp_path = GVP::smooth(gvp_path, state, discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
        }
    }

    return GVP::densify(gvp_path, discretization);
}


/************************
 *  LBKPIECE
 ***********************/
ompl::base::PlannerPtr LBKPIECE_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    std::shared_ptr<og::LBKPIECE1> pp = std::make_shared<og::LBKPIECE1>(si);
    pp->setRange(discretization*50);
    std::cout << "Range: " << pp->getRange() << "\n";
    return pp;
}

std::string LBKPIECE_Strategy::getName() const
{
    return "LBKPIECE_Strategy";
}

Path LBKPIECE_Strategy::smooth(Path gvp_path, State &state)
{
    std::mt19937 rng;
    rng.seed(42);
    
    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            gvp_path = GVP::smooth(gvp_path, state, discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
        }
    }

    return GVP::densify(gvp_path, discretization);
}


/************************
 *  PRM
 ***********************/
ompl::base::PlannerPtr PRM_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    std::shared_ptr<og::LazyPRM> pp = std::make_shared<og::LazyPRM>(si);
    pp->setRange(discretization*50);
    std::cout << "Range: " << pp->getRange() << "\n";
    return pp;
}

std::string PRM_Strategy::getName() const
{
    return "PRM_Strategy";
}

Path PRM_Strategy::smooth(Path gvp_path, State &state)
{
    std::mt19937 rng;
    rng.seed(42);
    
    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            gvp_path = GVP::smooth(gvp_path, state, discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
        }
    }

    return GVP::densify(gvp_path, discretization);
}


/************************
 *  STRIDE
 ***********************/
ompl::base::PlannerPtr STRIDE_Strategy::makePlanner(ompl::base::SpaceInformationPtr si)
{
    std::shared_ptr<og::STRIDE> pp = std::make_shared<og::STRIDE>(si);
    pp->setRange(discretization*50);
    std::cout << "Range: " << pp->getRange() << "\n";
    return pp;
}

std::string STRIDE_Strategy::getName() const
{
    return "STRIDE_Strategy";
}

Path STRIDE_Strategy::smooth(Path gvp_path, State &state)
{
    std::mt19937 rng;
    rng.seed(42);
    
    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            gvp_path = GVP::smooth(gvp_path, state, discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(gvp_path)));
        }
    }

    return GVP::densify(gvp_path, discretization);
}
