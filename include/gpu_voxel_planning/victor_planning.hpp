#ifndef VICTOR_PLANNING_HPP
#define VICTOR_PLANNING_HPP

#include "victor_validator.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "lazyrrt_fullpath.h"
#include "cost_rrtconnect.h"


namespace gpu_voxels_planner
{


    typedef ompl::base::ScopedState<> Goals;
    typedef std::shared_ptr<ompl::geometric::PathGeometric> Ompl_Path;
        
    class VictorPlanner
    {
    public:
        VictorPlanner(GpuVoxelsVictor* victor_model);

        virtual void initializePlanner() = 0;

        virtual void preparePlanner(ompl::base::ScopedState<> start, Goals goals) = 0;

        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                   Goals goals);

        virtual void setupSpaceInformation();
        
        virtual void post_planning_actions(ompl::base::PathPtr path) {(void) path;};

        VictorConfig toVictorConfig(ompl::base::ScopedState<> s);

        // ompl::base::ScopedState<> samplePointInRandomDirection(ompl::base::ScopedState<> start);
        
        Maybe::Maybe<Path> localControlConfig(VictorConfig start, VictorConfig goal);

        Maybe::Maybe<Path> localControlDouble(std::vector<double> start, std::vector<double> goal);

        virtual Maybe::Maybe<ompl::base::PathPtr> localControl(ompl::base::ScopedState<> start, Goals goal);

        virtual Maybe::Maybe<ompl::base::PathPtr> voxLocalControl(ompl::base::ScopedState<> start,
                                                                  Goals goals);

        ompl::base::PathPtr randomWiggle(ompl::base::ScopedState<> start);

        Path randomWiggleConfig(VictorConfig start);

        ompl::base::PathPtr iouWiggle(ompl::base::ScopedState<> start);

        Path iouWiggleConfig(VictorConfig start);

        double evaluateIouExploration(Path path);
        


    public:

        Maybe::Maybe<Path> planPathConfig(VictorConfig start, VictorConfig goal);
        
        Maybe::Maybe<Path> planPathDouble(std::vector<double> start, std::vector<double> goal);


        std::shared_ptr<VictorValidator> vv_ptr;
        std::shared_ptr<ompl::base::RealVectorStateSpace> space;


        std::shared_ptr<ompl::geometric::PathValidator> vppc;

        double controller_threshold;

        std::string name;
        
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
        virtual void initializePlanner() override;

        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                     Goals goals) override;

    };




    class VictorLazyRRTF : public VictorPlanner
    {
    public:
        VictorLazyRRTF(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;
        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;
        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                    Goals goals) override;
    protected:
        std::shared_ptr<VictorPathProbCol> pv_;
        double threshold;
    };




    class VictorPRM: public VictorPlanner
    {
    public:
        VictorPRM(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;

        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                     Goals goals) override;
    };

    class VictorRRTConnect: public VictorPlanner
    {
    public:
        VictorRRTConnect(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;

        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                     Goals goals) override;
    };



    class VictorSamplingRRTConnect: public VictorPlanner
    {
    public:
        VictorSamplingRRTConnect(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;

        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                     Goals goals) override;
        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;
    protected:
        std::shared_ptr<VictorPathProbCol> path_prob_validator;

    };

    
    class DiversePlanner: public VictorPlanner
    {
    public:
        DiversePlanner(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;

        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                     Goals goals) override;

        Maybe::Maybe<ompl::base::PathPtr> planSinglePathInSampledWorld(ompl::base::ScopedState<> start, Goals goals);

        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;

        bool sampleBlockingWorld(std::vector<ompl::base::PathPtr> paths);

        bool isPathSetValid(std::vector<ompl::base::PathPtr> paths);

        
    protected:
        std::shared_ptr<VictorPathProbCol> path_prob_validator;
        std::shared_ptr<VictorSampledWorldValidator> vv_sampled_world_ptr;

    };


    
    class VictorThresholdRRTConnect: public VictorPlanner
    {
    public:
        VictorThresholdRRTConnect(GpuVoxelsVictor* victor_model,
                                  bool use_vox, bool use_prob_col);
        virtual void initializePlanner() override;
        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;
        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                    Goals goals) override;
        
        virtual Maybe::Maybe<ompl::base::PathPtr> localControl(ompl::base::ScopedState<> start,
                                                               Goals goals) override;

        bool use_vox_;
        bool use_prob_col_;

    };



    class VictorMotionCostRRTConnect: public VictorPlanner
    {
    public:
        VictorMotionCostRRTConnect(GpuVoxelsVictor* victor_model);
        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;

        Maybe::Maybe<ompl::base::PathPtr> planAnytime(ompl::base::ScopedState<> start,
                                                      Goals goals);
        Maybe::Maybe<ompl::base::PathPtr> planUp(ompl::base::ScopedState<> start,
                                                 Goals goals);
        void smooth(ompl::base::PathPtr &path);
            
        
        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                    Goals goals) override;

        ompl::geometric::CostRRTConnect* rplanner_;

        double cost_upper_bound;
        bool use_anytime_planner{true};

    };

    class VictorVoxCostRRTConnect: public VictorMotionCostRRTConnect
    {
    public:
        VictorVoxCostRRTConnect(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;
        virtual Maybe::Maybe<ompl::base::PathPtr> localControl(ompl::base::ScopedState<> start,
                                                       Goals goals) override;
    };
    
        
    class VictorProbColCostRRTConnect: public VictorMotionCostRRTConnect
    {
    public:
        VictorProbColCostRRTConnect(GpuVoxelsVictor* victor_model);
        virtual void initializePlanner() override;
    };




    

    class VictorRRTStar: public VictorPlanner
    {
    public:
        VictorRRTStar(GpuVoxelsVictor* victor_model,
                      bool use_prob_cost);
        virtual void initializePlanner() override;
        virtual void preparePlanner(ompl::base::ScopedState<> start,
                                    Goals goals) override;
        virtual Maybe::Maybe<ompl::base::PathPtr> planPath(ompl::base::ScopedState<> start,
                                                           Goals goals) override;


        std::shared_ptr<VictorObjective> opt;
        bool use_prob_cost_;
    };


}

        
#endif
