#ifndef OMPL_STRATEGIES_HPP
#define OMPL_STRATEGIES_HPP

#include "gpu_voxel_planning/strategies/strategies.hpp"
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
// #include <ompl/geometric/planners/prm/SPARStwo.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "gpu_voxel_planning/params.hpp"


namespace GVP
{
    class OMPL_Strategy : public Strategy
    {
    public:
        double discretization;
        GpuVoxelRvizVisualizer* viz;
        int viz_id=0;
        
    public:
        OMPL_Strategy() : discretization(EDGE_DISCRETIZATION) {}
        
        Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) override;

    protected:
        std::shared_ptr<ompl::base::RealVectorStateSpace> makeSpace();

        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) = 0;
            
        bool isOmplStateValid(const ompl::base::State *ompl_state,
                              GVP::State &gvp_state);
        void vizPoint(VictorRightArmConfig& config, bool valid);
        

        virtual Path smooth(Path gvp_path, State &state) = 0;

    private:
        std::vector<double> previously_sampled_point;

    };


    class RRT_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };

    class SPARS_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };

    // class SPARSTWO_Strategy : public OMPL_Strategy
    // {
    // public:
    //     virtual std::string getName() const override;
    //     virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
    //     Path smooth(Path gvp_path, State &state) override;
    // };

    class BIT_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };

    class SST_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };


    class LBKPIECE_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };


    class PRM_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };

    class STRIDE_Strategy : public OMPL_Strategy
    {
    public:
        virtual std::string getName() const override;
        virtual ompl::base::PlannerPtr makePlanner(ompl::base::SpaceInformationPtr si) override;
        Path smooth(Path gvp_path, State &state) override;
    };
}



#endif
