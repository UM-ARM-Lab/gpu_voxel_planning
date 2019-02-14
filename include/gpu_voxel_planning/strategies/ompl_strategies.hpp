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
        Path applyTo(Scenario &scenario) override;
        
    protected:
        std::shared_ptr<ompl::base::RealVectorStateSpace> makeSpace();
            
        bool isOmplStateValid(const ompl::base::State *ompl_state,
                              const GVP::State &gvp_state);
    };


    class RRT_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
    };
}



#endif
