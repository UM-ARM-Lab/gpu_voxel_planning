#include "victor_local_controller.hpp"
#include "gpu_voxel_planning/ompl_utils.hpp"
#include "gpu_voxel_planning/helpers.hpp"
#include "gpu_voxel_planning/hacky_functions.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels_planner;
using namespace PathUtils;



VictorLocalController::VictorLocalController(GpuVoxelsVictor* victor_model)
{
    double torad=3.1415/180;
    space = std::make_shared<ob::RealVectorStateSpace>();
    for(int i=0; i<7; i++)
    {
        space->addDimension(right_joint_lower_deg[i]*torad, right_joint_upper_deg[i]*torad);
    }

    
    si_ = std::make_shared<ob::SpaceInformation>(space);

    victor_model_ = victor_model;

}



Path VictorLocalController::maxExpectedChsIG(std::vector<double> start_values,
                                             double max_motion,
                                             int num_samples)
{
    ob::ScopedState<> start = ompl_utils::toScopedState(start_values, space);
    double best_IG = 0;
    ob::ScopedState<> best_point(si_);
    
    std::vector<size_t> chs_sizes = victor_model_->chsSizes();
    for(int i=0; i<num_samples; i++)
    {
        ob::ScopedState<> new_point = ompl_utils::samplePointInRandomDirection(start, si_, max_motion);
        std::vector<ob::State*> query_path;
        size_t num_interp_points = max_motion / si_->getStateSpace()->getLongestValidSegmentLength();
        si_->getMotionStates(start.get(), new_point.get(), query_path, num_interp_points, true, true);

        
        victor_model_->getMap(VICTOR_QUERY_MAP)->clearMap();
        for(const ob::State *s: query_path)
        {
            const double *values = s->as<ob::RealVectorStateSpace::StateType>()->values;
            victor_model_->addQueryState(victor_model_->toVictorConfig(values));
        }
        std::vector<size_t> chs_overlaps = victor_model_->countCHSCollisions();

        si_->freeStates(query_path);


        double p_no_col = 1 - pCollision(chs_overlaps, chs_sizes);
        // double expected_IG = -p_no_col * log2(p_no_col);
        double expected_IG = p_no_col * IG(chs_overlaps, chs_sizes);


        // std::cout << "p_no_col: " << p_no_col << "  IG: " << expected_IG << "\n";
        // victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
        // waitForKeypress();


        if(expected_IG > best_IG)
        {
            best_IG = expected_IG;
            best_point = new_point;
        }
    }

    std::cout << "best IG: " << best_IG << "\n";
    // victor_model_->resetQuery();
    // victor_model_->addQueryState(victor_model_->toVictorConfig(best_point.reals().data()));
    // victor_model_->gvl->visualizeMap(VICTOR_QUERY_MAP);
    // waitForKeypress();


    Path path;
    path.push_back(start.reals());
    path.push_back(best_point.reals());
    return path;
    
}
