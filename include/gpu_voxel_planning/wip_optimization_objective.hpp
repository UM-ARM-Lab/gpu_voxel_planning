#ifndef WIP_OPTIMIZAION_OBJECTIVE
#define WIP_OPTIMIZAION_OBJECTIVE


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <gpu_voxels/GpuVoxels.h>



class WipOptimizationObjective : public ompl::base::OptimizationObjective
{
public:
    WipOptimizationObjective(ompl::base::SpaceInformationPtr si,
                             gpu_voxels::GpuVoxelsSharedPtr gvl);


    virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;
    
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1,
                                        const ompl::base::State *s2) const;

    gpu_voxels::GpuVoxelsSharedPtr gvl_;
};


#endif
