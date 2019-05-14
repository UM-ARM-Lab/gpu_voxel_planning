#ifndef GVP_REAL_SCENARIOS_HPP
#define GVP_REAL_SCENARIOS_HPP

#include "scenarios/scenarios.hpp"

namespace GVP
{
    
    class RealScenario : public Scenario
    {
    public:
        RealState s;
        ObstacleConfiguration known_obstacles;
        
        RealScenario();

        void initFakeVictor(RosInterface &ri);
        // virtual void setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp);
        virtual void setPrior(BeliefParams bp);

        virtual void validate();

        virtual void viz(const GpuVoxelRvizVisualizer& viz) override;

        virtual State& getState() override
        {
            return s;
        }

        virtual const State& getState() const override
        {
            return s;
        }

        virtual RealState& getRealState()
        {
            return s;
        }

        virtual const RealState& getRealState() const
        {
            return s;
        }

        void addLeftArm();

    };



    /****************************************
     **         Table With Box
     ****************************************/
    class RealTable : public RealScenario
    {
    public:
        const std::string name;

        RealTable(BeliefParams bp);

        virtual std::string getName() const override
        {
            return name;
        }

        Object getTable();
    };
}


#endif