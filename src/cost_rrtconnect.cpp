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

#include "cost_rrtconnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"


ompl::geometric::CostRRTConnect::CostRRTConnect(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "CostRRTConnectIntermediate" : "CostRRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CostRRTConnect::setRange, &CostRRTConnect::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;

}

ompl::geometric::CostRRTConnect::~CostRRTConnect()
{
    freeMemory();
}

void ompl::geometric::CostRRTConnect::setup()
{
    Planner::setup();
    path_cost = std::numeric_limits<double>::max();
    tools::SelfConfig sc(si_, getName());
    // maxDistance_ = si_->getStateSpace()->getLongestValidSegmentFraction() * 10;
    
    maxDistance_ = si_->getStateSpace()->getLongestValidSegmentLength() * 20;
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::CostRRTConnect::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::CostRRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::CostRRTConnect::GrowState
ompl::geometric::CostRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                          Motion *rmotion, bool limit)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (limit && (d > maxDistance_))
    {
        // std::cout << "limiting to " << maxDistance_ << "\n";
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* check if we have moved at all */
        if (si_->distance(nmotion->state, tgi.xstate) < std::numeric_limits<double>::epsilon())
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
    }

    // std::cout << "Distance of extend: " << si_->distance(nmotion->state, dstate) << "\n";
    // std::cout << "valid segment count: " << si_->getStateSpace()->validSegmentCount(nmotion->state, dstate) << "\n";
    // std::cout << "longest valid seg: "<< si_->getStateSpace()->getLongestValidSegmentLength() << "\n";
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it
    // receives as argument is valid,
    // so we check that one first
    if (addIntermediateStates_)
    {
        // std::cout << "Adding intermediate states seems to have some bugs\n";
        // assert(false);
        // std::string treename = tgi.start ? "start tree" : "goal tree";
        // std::cout << "Adding intermediate states to " << treename << "\n";
        // std::vector<base::State *> states;
        // const unsigned int count =
        //     1 + si_->distance(nmotion->state, dstate) / si_->getStateValidityCheckingResolution();
        // ompl::base::State *nstate = nmotion->state;
        // std::cout << "Motion state count " << count << "\n";
        // std::cout << "Distance " << si_->distance(nmotion->state, dstate) << "\n";
        // if (tgi.start)
        //     si_->getMotionStates(nstate, dstate, states, count, true, true);
        // else
        // {
        //     si_->getStateValidityChecker()->isValid(dstate) &&
        //         si_->getMotionStates(dstate, nstate, states, count, true, true);
        //     // std::cout << "but only for single state\n";
        // }
        // if (states.empty())
        //     return TRAPPED;
        // bool adv = si_->distance(states.back(), tgi.start ? dstate : nstate) <= 0.01;
        

        // reach = reach && adv;
        // si_->freeState(states[0]);
        // Motion *motion;
        // for (std::size_t i = 1; i < states.size(); i++)
        // {
        //     if (adv)
        //     {
        //         /* create a motion */
        //         motion = new Motion;
        //         motion->state = states[i];
        //         motion->parent = nmotion;
        //         motion->root = nmotion->root;
        //         nmotion->children.push_back(motion);
        //         tgi.xmotion = motion;
        //         nmotion = motion;
        //         tree->add(motion);
        //     }
        //     else
        //         si_->freeState(states[i]);
        // }
        // if (reach)
        //     return REACHED;
        // else if (adv)
        //     return ADVANCED;
        // else
        //     return TRAPPED;







        // std::cout << "Adding intermediate states\n";
        bool validMotion =
            tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                        si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

        if(!validMotion)
        {
            std::cout << "invalid motion\n";
            return TRAPPED;
        }
        
        auto path(std::make_shared<PathGeometric>(si_));
        path->append(nmotion->state);
        path->append(dstate);

        // std::cout << "path has " << path->getStates().size();
        path->interpolate();
        // std::cout << ". After interpolating " << path->getStates().size() << "\n";

        // std::vector<base::State*> path;
        // path.push_back(nmotion->state);
        // path.push_back(dstate);
        // size_t collision_index;
        pv_->setProbabilityThreshold(threshold);
        std::vector<double> costs;
        // pv_->do_delay = true;
        // std::cout << "rrt threshold: " << threshold << "\n";
        double new_motion_cost = pv_->getPathCost(path->getStates(), costs);
        if(pv_->do_delay)
            std::cout << "costs size: " << costs.size() << " path size: " << path->getStates().size() << "\n";


        double nd = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);
        double nn_cost_from_root = nmotion->cost_from_root;
        int count = 0;
        for(size_t i=0; i < costs.size(); i+=1)
        {
            if(!(costs[i] <= threshold))
            {
                std::cout << "costs: " << costs[i] << "\n";
            }
            assert(costs[i] <= threshold);            
            double new_cost_from_root = accumulateCost(nn_cost_from_root, costs[i]);

            if(new_cost_from_root >= threshold)
            {
                // std::cout << "total cost for added motion exceed threshold. Returing trapped\n";
                return TRAPPED;
            }

            assert(i < nd);

            
            Motion *motion = new Motion(si_);
            // si_->getStateSpace()->interpolate(nmotion->state,
            //                                   dstate, (double)i / nd, motion->state);
            // motion->state = si_->copyState(motion->state, path->getState(i+1));
            si_->copyState(motion->state, path->getState(i+1));
            // si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->root = nmotion->root;
            motion->cost_from_root = new_cost_from_root;
            nmotion->children.push_back(motion);
            tgi.xmotion = motion;
            nmotion = motion;
            tree->add(motion);
            count++;
            
            // std::cout << "Adding motion with cost " << new_cost_from_root << "\n";
        }
        // std::cout << "Added " << count << " motions. Reached? " << reach << "  Trapped? " << (new_motion_cost > threshold) << "\n";

        // std::cout << "cost sizes: " << costs.size();
        // std::cout << " new_motion_cost: " << new_motion_cost;
        // std::cout << " threshold: " << threshold << "\n";
        
        if(new_motion_cost > threshold || costs.size() == 0)
        {
            return TRAPPED;
        }
            
        if (reach)
            return REACHED;
        else
            return ADVANCED;
     

    }
    else // dont add intermediate
    {
        // std::cout << "Not adding intermediate states\n";
        bool validMotion =
            tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                        si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

        if(!validMotion)
        {
            return TRAPPED;
        }

        

        std::vector<base::State*> path;
        path.push_back(dstate);
        path.push_back(nmotion->state);
        size_t collision_index;
        pv_->setProbabilityThreshold(threshold);
        // pv_->do_delay = true;
        double new_motion_cost = pv_->getPathCost(path, collision_index);
        double new_cost_from_root = accumulateCost(nmotion->cost_from_root, new_motion_cost);
            
        if (new_cost_from_root < threshold)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->root = nmotion->root;
            motion->cost_from_root = new_cost_from_root;
            nmotion->children.push_back(motion);
            tgi.xmotion = motion;

            
            tree->add(motion);
            if (reach)
                return REACHED;
            else
                return ADVANCED;
        }
        else
            return TRAPPED;
    }
}

/*
 *  Accumulation of cost, in this case probabilities of collision.
 *  Collisions probabilities are assumed to be indep
 */
double ompl::geometric::CostRRTConnect::accumulateCost(double cost_1, double cost_2)
{
    return cost_1 + cost_2;
}


ompl::base::PlannerStatus ompl::geometric::CostRRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // std::cout << "pis_ has " << pis_.haveMoreStartStates() << "\n";
    while (const base::State *st = pis_.nextStart())
    {
        // std::cout << "adding new state to motion\n";
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);


        // std::cout << "extend\n";
        GrowState gs = growTree(tree, tgi, rmotion, true);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            // std::cout << "connect\n";
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
            {
                // std::cout << "growing to connect\n";
                gsc = growTree(otherTree, tgi, rmotion);
            }

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {

                std::vector<Motion*> motionPath;
                makePath(startMotion, goalMotion, motionPath);
                
                if(validateFullPath(motionPath))
                {
                    auto path(std::make_shared<PathGeometric>(si_));
                    path->getStates().reserve(motionPath.size());
                    for(auto &m: motionPath)
                    {
                        path->append(m->state);
                    }

                    pdef_->addSolutionPath(path, false, 0.0, getName());
                    solved = true;
                    break;
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::CostRRTConnect::validateFullPath(std::vector<Motion*> &mpath)
{
    std::cout << "Validating full path\n";
    pv_->setProbabilityThreshold(threshold);
    size_t collision_index;
    std::vector<ompl::base::State*> spath;
    for(auto m: mpath)
    {
        spath.push_back(m->state);
    }
    double full_motion_cost = pv_->getPathCost(spath, collision_index);
    if(full_motion_cost < threshold)
    {
        std::cout << "Path validated\n";
        path_cost = full_motion_cost;
        return true;
    }


    double max_cost = 0.0;
    Motion *worst_motion;
    for(auto m: mpath)
    {
        if(m->parent == nullptr)
        {
            // Start/Goal have no parents
            continue;
        }
        std::vector<ompl::base::State*> step;
        step.push_back(m->state);
        step.push_back(m->parent->state);
        size_t col_index;
        double cost = pv_->getPathCost(step, col_index);
        if(cost > max_cost)
        {
            max_cost = cost;
            worst_motion = m;
        }
    }
    std::cout << "Invalid path found, " << full_motion_cost << " exceeds " << threshold << ". Removing motion with cost " << max_cost << "\n";
    removeMotion(worst_motion);
    
    return false;
}

void ompl::geometric::CostRRTConnect::removeMotion(Motion *motion)
{
    std::cout << "removing motion\n";
    tStart_->remove(motion);
    tGoal_->remove(motion);

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


void ompl::geometric::CostRRTConnect::makePath(Motion *startMotion,
                                               Motion *goalMotion,
                                               std::vector<Motion*> &motionPath)
{
    // it must be the case that either the start tree or the goal tree has made some progress
    // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
    // on the solution path
    if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

    /* construct the solution path */
    Motion *solution = startMotion;
    std::vector<Motion *> mpath1;
    while (solution != nullptr)
    {
        mpath1.push_back(solution);
        solution = solution->parent;
    }

    solution = goalMotion;
    std::vector<Motion *> mpath2;
    while (solution != nullptr)
    {
        mpath2.push_back(solution);
        solution = solution->parent;
    }


    // motionPath.reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i)
        motionPath.push_back(mpath1[i]);
    for (auto &i : mpath2)
        motionPath.push_back(i);
}

void ompl::geometric::CostRRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = boost::lexical_cast<std::string>(distanceBetweenTrees_);
}





/***********************************************
 *         Probability of Collision cost
 ***********************************************/

ompl::geometric::ProbColRRTConnect::ProbColRRTConnect(const base::SpaceInformationPtr &si,
                                                      bool addIntermediateStates)
    : CostRRTConnect(si, addIntermediateStates)
{
    std::cout << "AddIntermediatestates: " << addIntermediateStates << "\n";
}



/*
 *  Accumulation of cost, in this case probabilities of collision.
 *  Collisions probabilities are assumed to be indep
 */
double ompl::geometric::ProbColRRTConnect::accumulateCost(double cost_1, double cost_2)
{
    return 1.0 - (1.0 - cost_1)*(1.0 - cost_2);
}

