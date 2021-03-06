/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "lazyrrt_fullpath.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <cassert>
#include <algorithm>
#include <ompl/base/spaces/SE3StateSpace.h>

ompl::geometric::LazyRRTF::LazyRRTF(const base::SpaceInformationPtr &si) : base::Planner(si, "LazyRRTF")
{
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &LazyRRTF::setRange, &LazyRRTF::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &LazyRRTF::setGoalBias, &LazyRRTF::getGoalBias, "0.:.05:1.");
}

ompl::geometric::LazyRRTF::~LazyRRTF()
{
    freeMemory();
}

void ompl::geometric::LazyRRTF::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}


void ompl::geometric::LazyRRTF::setPathValidator(std::shared_ptr<PathValidator> path_validator)
{
    path_validator_ = path_validator;
}


void ompl::geometric::LazyRRTF::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::LazyRRTF::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void printState(ompl::base::State *s)
{
    const double *values = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for(int i=0; i<7; i++)
    {
        std::cout << values[i] << ", ";
    }
    std::cout << "\n";
}

ompl::base::PlannerStatus ompl::geometric::LazyRRTF::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    base::State *start_state;
    
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->valid = true;
        nn_->add(motion);
        start_state = motion->state;
        std::cout << "Start state: ";
        printState(motion->state);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    double distsol = -1.0;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    bool solutionFound = false;
    bool goalSampled;

    while (!ptc && !solutionFound)
    {
        // goalBias_ = 0.001;
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
            goalSampled = true;
            std::cout << "goal sampled\n";
        }
        else
        {
            sampler_->sampleUniform(rstate);
            goalSampled = false;
            // std::cout << "Sampling state\n";
        }

        /* find closest state in the tree */

        Motion *nmotion = nn_->nearest(rmotion);
        assert(nmotion != rmotion);
        base::State *dstate = rstate;
        if(goalSampled)// && nmotion->state==start_state)
        {
            // std::cout << nmotion->state << "\n";
            // std::cout << start_state << "\n";
            // // std::cout << "Start state is nearest to goal\n";
            // std::cout << "nn size: " << nn_->size() << "\n";
            std::cout << "Nearest to goal\n";
            printState(nmotion->state);
            // std::cout << "start is nearest to goal\n";
        }
        

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        std::cout << "Adding motion to ";
        printState(dstate);

        /* create a motion */
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        nmotion->children.push_back(motion);
        nn_->add(motion);

        double dist = 0.0;
        if (goal->isSatisfied(motion->state, &dist))
        {
            distsol = dist;
            solution = motion;
            // solutionFound = true;
            lastGoalMotion_ = solution;

            // Check that the solution is valid:
            // construct the solution path
            std::vector<Motion *> mpath;
            std::vector<ompl::base::State*> spath;
            // std::cout << "motion:\n";
            while (solution != nullptr)
            {
                mpath.push_back(solution);
                spath.push_back(solution->state);
                solution = solution->parent;
                // const double *values = solution->state->as<base::RealVectorStateSpace::StateType>()->values;
                // for(int i=0; i<7; i++)
                // {
                //     std::cout << values[i] << ", ";
                // }
                // std::cout << "\n";

            }

            std::reverse(std::begin(mpath), std::end(mpath));
            std::reverse(std::begin(spath), std::end(spath));
            

            // // check each segment along the path for validity
            // for (int i = mpath.size() - 1; i >= 0 && solutionFound; --i)
            //     if (!mpath[i]->valid)
            //     {
            //         if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
            //             mpath[i]->valid = true;
            //         else
            //         {
            //             removeMotion(mpath[i]);
            //             solutionFound = false;
            //             lastGoalMotion_ = nullptr;
            //         }
            //     }

            size_t collision_index;
            // std::cout << "Checking path...";
            // std::cout << "nn_size: " << nn_->size() << "\n";
            // std::cout << "mpath size: " << mpath.size() << "\n";
            solutionFound = path_validator_->checkPath(spath, collision_index);
            // std::cout << "done checking path\n";

            if (solutionFound)
            {
                // std::cout << "Adding solution path...";
                // set the solution path
                std::reverse(std::begin(mpath), std::end(mpath));
                auto path(std::make_shared<PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                    path->append(mpath[i]->state);

                pdef_->addSolutionPath(path, false, distsol, getName());
                // std::cout << "Done adding solution path\n";
            }
            else
            {
                // std::cout << "Removing Motion...";
                // for (size_t i = collision_index + 2; i < mpath.size(); i++)
                // {
                size_t i = collision_index+1;
                assert(i < mpath.size());
                // std::cout << "nn size " << nn_->size() << "\n";
                // std::cout << "removing motion " << i << "\n";
                removeMotion(mpath[i]);
                // std::cout << "nn size " << nn_->size() << "\n";
                lastGoalMotion_ = nullptr;
                // std::cout << "Done removing motion\n";
                // }
            }
            
        }
    }

    si_->freeState(xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return solutionFound ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LazyRRTF::removeMotion(Motion *motion)
{
    nn_->remove(motion);

    /* remove self from parent list */

    if (motion->parent != nullptr)
    {
        for (unsigned int i = 0; i < motion->parent->children.size(); ++i)
            if (motion->parent->children[i] == motion)
            {
                motion->parent->children.erase(motion->parent->children.begin() + i);
                break;
            }
    }

    /* remove children */
    for (auto &i : motion->children)
    {
        i->parent = nullptr;
        removeMotion(i);
    }

    if (motion->state != nullptr)
        si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::LazyRRTF::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state, 1));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent != nullptr ? motion->parent->state : nullptr),
                         base::PlannerDataVertex(motion->state));

        data.tagState(motion->state, motion->valid ? 1 : 0);
    }
}
