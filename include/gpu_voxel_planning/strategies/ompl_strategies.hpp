#ifndef OMPL_STRATEGIES_HPP
#define OMPL_STRATEGIES_HPP

#include "strategies/strategies.hpp"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace GVP
{
    class OMPL_Strategy : public Strategy
    {
    public:
        double discretization;
        
    public:
        OMPL_Strategy() : discretization(0.02) {}
        
        Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) override;
        
    protected:
        std::shared_ptr<ompl::base::RealVectorStateSpace> makeSpace();

        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) = 0;
            
        bool isOmplStateValid(const ompl::base::State *ompl_state,
                              GVP::State &gvp_state);

        virtual Path smooth(Path gvp_path, State &state) = 0;


    };


    class RRT_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };

    class BIT_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };
}



#endif
