
#ifndef OMPL_PATH_PATH_SMOOTHING_H
#define OMPL_PATH_PATH_SMOOTHING_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include "path_validator.h"

namespace ompl
{
    namespace geometric
    {
        class PathSmoother
        {
        public:
            PathSmoother(ompl::base::SpaceInformationPtr si,
                           PathValidator* pv)
            {
                si_ = si;
                pv_ = pv;
                eps = 0.001;
            }

            void shortcutPath(PathGeometric &path, size_t num_trials);

            void singleShortcut(PathGeometric &path);
            
        protected:
            void sampleInd(int &start, int &end, int max_exclusive);
            
            ompl::base::SpaceInformationPtr si_;
            PathValidator* pv_;
            double cur_cost;
            double eps;
            std::mt19937 rng;
        };
    }
}


#endif
