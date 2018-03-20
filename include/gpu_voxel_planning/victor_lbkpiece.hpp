#include "victor_planning.hpp"

namespace gpu_voxels_planner
{
    class VictorLBKPiece: public VictorPlanner
    {
    public:
        VictorLBKPiece();
        virtual void setup_planner();
        
        virtual void prepare_planner(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);
    };
}
