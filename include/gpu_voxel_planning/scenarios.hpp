#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP


#include "state.hpp"
#include "gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class Scenario
    {
    public:
        VictorRightArm victor;
        robot::JointValueMap goal_config;
        virtual State& getState() = 0;
        virtual const State& getState() const = 0;
        virtual std::string getName() const = 0;
        
        virtual bool completed() const
        {
            return VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal_config);
        }

        virtual void viz(const GpuVoxelRvizVisualizer& viz)
        {
        }

        Scenario(){}
    };

    
    class SimulationScenario : public Scenario
    {
    public:
        SimulationState s;
        ObstacleConfiguration true_obstacles;
        ObstacleConfiguration known_obstacles;
        ObstacleConfiguration unknown_obstacles;
        
        SimulationScenario();
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
        SlottedWall(bool all_known);

        std::string getName() const
        {
            return "SlottedWall";
        }

        Object getSlottedWall();
    };



    /****************************************
     **         Bookshelf
     ****************************************/
    class Bookshelf : public SimulationScenario
    {
        const std::string name;
    public:
        Bookshelf(bool all_known);

        std::string getName() const
        {
            return "Bookshelf";
        }

        Object getBookshelf();

        Object getTable();
    };

}


#endif
