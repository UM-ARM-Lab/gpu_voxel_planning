#ifndef GVP_SIMULATION_SCENARIOS_HPP
#define GVP_SIMULATION_SCENARIOS_HPP

#include "scenarios/scenarios.hpp"

namespace GVP
{
    
    class SimulationScenario : public Scenario
    {
    public:
        SimulationState s;
        ObstacleConfiguration true_obstacles;
        ObstacleConfiguration known_obstacles;
        ObstacleConfiguration unknown_obstacles;

        std::string belief_name;
        
        SimulationScenario();

        void initFakeVictor(RosInterface &ri);
        virtual void setPrior(ObstacleConfiguration &unknown_obstacles, BeliefParams bp);

        virtual void validate();

        virtual void viz(const GpuVoxelRvizVisualizer& viz) override;

        virtual DenseGrid& getTrueObstacles()
        {
            return true_obstacles.occupied;
        }

        virtual const DenseGrid& getTrueObstacles() const
        {
            return true_obstacles.occupied;
        }

        virtual SimulationState& getSimulationState()
        {
            return s;
        }

        virtual const SimulationState& getSimulationState() const
        {
            return s;
        }

        virtual State& getState() override
        {
            return s;
        }

        virtual const State& getState() const override
        {
            return s;
        }

        void addLeftArm();

    protected:
        void combineObstacles();
    };



    /****************************************
     **         Table With Box
     ****************************************/
    class TableWithBox : public SimulationScenario
    {
    public:
        Vector3f cavecorner;
        Vector3f caveheight;
        Vector3f cavetopd;
        Vector3f cavesidedim;
        Vector3f cavesideoffset;
        const std::string name;

        TableWithBox(BeliefParams bp, bool table_known=true, bool visible_cave_known=true,
                     bool full_cave_known=false);

        virtual std::string getName() const override
        {
            return name;
        }

        Object getTable();

        void setCaveDims();

        Object getVisibleCave();

        Object getCaveBack();
    };



    /****************************************
     **         SlottedWall
     ****************************************/
    class SlottedWall : public SimulationScenario
    {
        const std::string name;
    public:
        SlottedWall(BeliefParams bp);

        std::string getName() const
        {
            return "SlottedWall";
        }
        Object getFrontWall() const;
        Object getSlottedWall() const;
    };



    /****************************************
     **         Bookshelf
     ****************************************/
    class Bookshelf : public SimulationScenario
    {
        const std::string name;
    public:
        Bookshelf(BeliefParams bp);

        std::string getName() const
        {
            return "Bookshelf";
        }

        Object getBookshelf();

        Object getTable();
    };


    /****************************************
     **      Close Wall
     ****************************************/
    class CloseWall : public SimulationScenario
    {
        const std::string name;
    public:
        CloseWall(BeliefParams bp);

        std::string getName() const
        {
            return "CloseWall";
        }

        Object getCloseWall();
    };

}


#endif
