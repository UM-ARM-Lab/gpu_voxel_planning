
#ifndef OMPL_PATH_VALIDATOR_H
#define OMPL_PATH_VALIDATOR_H

#include <ompl/geometric/planners/PlannerIncludes.h>


namespace ompl
{
    namespace geometric
    {
        class PathValidator
        {
        public:
            PathValidator(ompl::base::SpaceInformationPtr si)
            {
                si_ = si;
            }

            
            /*
             *  Checks an entire path. If there is a collision, sets "collision_index" to 
             *   be the index of the nodes at the start of the motion with the collision
             */
            virtual bool checkPath(const std::vector<ompl::base::State*> path,
                                   size_t &collision_index) = 0;
        protected:
            ompl::base::SpaceInformationPtr si_;
        };
    }
}


#endif
