#ifndef VICTOR_PLANNING_HPP
#define VICTOR_PLANNING_HPP

#include "victor_validator.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>


namespace gpu_voxels_planner
{
    class VictorPlanner
    {
    public:
        VictorPlanner(GpuVoxelsVictor* victor_model);

        virtual void setup_planner() = 0;

        virtual void prepare_planner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal) = 0;

        virtual void setupSpaceInformation();
        
        virtual void post_planning_actions(ompl::base::PathPtr path) {(void) path;};

    public:
        Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                   ompl::base::ScopedState<> goal);
        Maybe::Maybe<Path> planPathDouble(std::vector<double> start, std::vector<double> goal);

        Path omplPathToDoublePath(ompl::geometric::PathGeometric* ompl_path);
        std::shared_ptr<VictorValidator> vv_ptr;
        std::shared_ptr<ompl::base::RealVectorStateSpace> space;

        
    protected:

        std::shared_ptr<ompl::base::SpaceInformation> si_;
        std::shared_ptr<ompl::geometric::PathSimplifier> simp_;
        std::shared_ptr<ompl::base::ProblemDefinition> pdef_;
        // std::shared_ptr<ompl::geometric::LBKPIECE1> planner_;
        // std::shared_ptr<ompl::geometric::TRRT> planner_;
        std::shared_ptr<ompl::base::OptimizationObjective> objective_;
        std::shared_ptr<ompl::base::Planner> planner_;
        GpuVoxelsVictor* victor_model_;
    };



    class VictorLBKPiece: public VictorPlanner
    {
    public:
        VictorLBKPiece(GpuVoxelsVictor* victor_model);
        virtual void setup_planner() override;

        virtual void prepare_planner(ompl::base::ScopedState<> start,
                                     ompl::base::ScopedState<> goal) override final;
    };

}
        
#endif
