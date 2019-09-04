#ifndef GVP_REAL_SCENARIOS_HPP
#define GVP_REAL_SCENARIOS_HPP

#include "gpu_voxel_planning/scenarios/scenarios.hpp"

namespace GVP
{
    
    class RealScenario : public Scenario
    {
    public:
        RealState s;
        ObstacleConfiguration known_obstacles;
        ObstacleConfiguration unknown_obstacles;

        std::string belief_name;
        
        RealScenario();

        void initFakeVictor(RosInterface &ri);
        virtual void setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp);
        // virtual void setPrior(BeliefParams bp);

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

        DenseGrid loadPointCloudFromFile();

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

    /****************************************
     **         Emtpy
     ****************************************/
    class RealEmpty : public RealScenario
    {
    public:
        const std::string name;

        RealEmpty(BeliefParams bp);

        virtual std::string getName() const override
        {
            return name;
        }

        Object getTable();
    };
}


#endif
