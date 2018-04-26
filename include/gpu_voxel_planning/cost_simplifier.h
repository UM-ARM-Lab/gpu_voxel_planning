
#ifndef OMPL_PATH_COST_SIMPLIFIER_H
#define OMPL_PATH_COST_SIMPLIFIER_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include "path_validator.h"

namespace ompl
{
    namespace geometric
    {
        class CostSimplifier
        {
        public:
            CostSimplifier(ompl::base::SpaceInformationPtr si,
                           PathValidator* pv)
            {
                si_ = si;
                pv_ = pv;
            }

            void shortcutPath(PathGeometric &path, size_t num_trials);

            void singleShortcut(PathGeometric &path);
            
        protected:
            ompl::base::SpaceInformationPtr si_;
            PathValidator* pv_;
        };
    }
}


#endif
