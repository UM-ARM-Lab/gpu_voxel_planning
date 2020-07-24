#ifndef GVP_SIMULATION_SCENARIOS_HPP
#define GVP_SIMULATION_SCENARIOS_HPP

#include "gpu_voxel_planning/scenarios/scenarios.hpp"

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

        // void addRightArm();

    protected:
        void combineObstacles();
    };

    /****************************************
     **         Empty
     ****************************************/
    class Empty : public SimulationScenario
    {
        const std::string name;
        
    public:
        Empty(BeliefParams bp);

        std::string getName() const
        {
            return "Empty";
        }
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

     /****************************************
     **      Glen's Scenario1
     ****************************************/

    
    class GlenScenario1 : public SimulationScenario
    {
        const std::string name;
    public:
        GlenScenario1(BeliefParams bp);

        std::string getName() const
        {
            return "GlenScenario1";
        }

        Object getObstacles();

        AABB getAABBFromBounds(std::vector<double> bounds)
        {
            double xoff = 1.0;
            double yoff = 1.3;
            double zoff = 1.0;
            Vector3f lower(bounds[0] + xoff, bounds[2] + yoff, bounds[4] + zoff);
            Vector3f upper(bounds[1] + xoff, bounds[3] + yoff, bounds[5] + zoff);
            return AABB(lower, upper);
        }

    };

}


#endif
