#ifndef GPU_PLANNING_STATE_HPP
#define GPU_PLANNING_STATE_HPP

#include "robot/robot_model.hpp"
#include "robot/robot_helpers.hpp"
#include <stdexcept>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/math_helpers.hpp>
#include "beliefs/beliefs.hpp"
#include "ros_interface/ros_interface.hpp"
#include "arc_utilities/stl_wrappers.hpp"

namespace GVP
{
    class State
    {
    public:
        double accumulated_cost = 0;
        
        Robot &robot;
        DenseGrid robot_self_collide_obstacles;
        DenseGrid known_obstacles;
        std::unique_ptr<Belief> bel;

        robot::JointValueMap current_config;

        State(Robot &robot) : robot(robot)
        {
        }

        State(const State& s) : State(s.robot)
        {
            robot_self_collide_obstacles = s.robot_self_collide_obstacles;
            known_obstacles = s.known_obstacles;
            current_config = s.current_config;
        }

        VictorRightArmConfig getCurConfig() const
        {
            return VictorRightArmConfig(current_config);
        }


        bool isPossiblyValid(const VictorRightArmConfig &c)
        {
            robot.set(c.asMap());
            if(robot.occupied_space.overlapsWith(&robot_self_collide_obstacles))
            {
                return false;
            }

            if(robot.occupied_space.overlapsWith(&known_obstacles))
            {
                return false;
            }
            return true;
        }

        double calcProbFree(const DenseGrid &volume)
        {
            PROFILE_START("CalcProbFreeKnown");
            if(robot_self_collide_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFreeKnown");
                return 0.0;
            }

            if(known_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFreeKnown");
                return 0.0;
            }
            PROFILE_RECORD("CalcProbFreeKnown");

            return bel->calcProbFree(volume);
        }

        double calcProbFree(const VictorRightArmConfig &c)
        {
            robot.set(c.asMap());
            return calcProbFree(robot.occupied_space);
        }

        void updateFreeSpace(const DenseGrid &new_free)
        {
            bel->updateFreeSpace(new_free);
        }

        void updateConfig(const robot::JointValueMap &jvm)
        {
            for(const auto& kv: jvm)
            {
                current_config[kv.first] = kv.second;
            }
        }

        State sample()
        {
            PROFILE_START("Sample_state");
            State sampled(robot);
            sampled.robot_self_collide_obstacles = robot_self_collide_obstacles;
            sampled.known_obstacles = bel->sampleState();
            sampled.known_obstacles.add(&known_obstacles);
            return sampled;
            PROFILE_RECORD("Sample_state");
        }

        virtual ~State() = default;
    };





    
    /*****************************************
     *      State for Simulation Trials
     ***************************************/
    class SimulationState : public State
    {
    public:
        SimulationState(Robot& robot) : State(robot) {}

        /* Simulated move of a short segment (one discretization unit)
         * Returns true if no collision, false if collision
         */
        bool move(const VictorRightArmConfig &c, const DenseGrid &true_world, RosInterface& ri)
        {
            PROFILE_START("simulation_state_move");
            accumulated_cost += EigenHelpers::Distance(VictorRightArmConfig(current_config).asVector(),
                                                       c.asVector());
            robot.set(c.asMap());
            if(robot.occupied_space.overlapsWith(&true_world))
            {
                bel->updateCollisionSpace(robot, getFirstLinkInCollision(robot, true_world));

                robot.set(current_config);
                return false;
            }
            ri.setRightArm(c);

            updateConfig(c.asMap());
            PROFILE_START("Update belief from free obs");
            updateFreeSpace(robot.occupied_space);
            PROFILE_RECORD("Update belief from free obs");
            PROFILE_RECORD("simulation_state_move");
            return true;
        }
    };


    /*****************************************
     *      State for Real Robot Trials
     ***************************************/
    class RealState : public State
    {
    public:
        RealState(Robot& robot) : State(robot) {}

        /*
         *  Moves robot either to new config, or stops at collision.
         *   Unlike SimulationState moves, these moves can be long
         */
        bool move(const std::vector<VictorRightArmConfig> &path, RosInterface& ri)
        {
            PROFILE_START("real_state_move");
            // accumulated_cost += EigenHelpers::Distance(VictorRightArmConfig(current_config).asVector(),
            //                                            c.asVector());
            gpu_voxel_planning::AttemptPathResultResponse resp = ri.moveRightArm(path);

            if(resp.ci.collided)
            {
                //TODO: handle physical robot case of collision
                DenseGrid sv;
                for(const auto free_point: resp.ci.free_path.points)
                {
                    robot.set(VictorRightArmConfig(free_point.positions).asMap());
                    sv.add(&robot.occupied_space);
                }
                updateFreeSpace(sv);

                robot.set(VictorRightArmConfig(resp.ci.collision_path.points.back().positions).asMap());

                std::cout << "Collision link: " << resp.ci.collision_links.front() << "\n";


                int first_col_link = arc_std::find(robot.getLinkNames(), resp.ci.collision_links.front());

                std::cout << "first col link: " << first_col_link << "\n";

                if(first_col_link < 0)
                {
                    throw std::logic_error("link in collision not found");
                }

                
                bel->updateCollisionSpace(robot, first_col_link);
                
                // throw std::logic_error("Not implemented exception");
                return false;
            }
            
            //TODO: update entire swept volume
            PROFILE_START("Update belief from free obs");
            DenseGrid sv;
            for(const auto c: path)
            {
                robot.set(c.asMap());
                sv.add(&robot.occupied_space);
            }
            updateFreeSpace(sv);
            PROFILE_RECORD("Update belief from free obs");


            updateConfig(path.back().asMap());
            PROFILE_RECORD("real_state_move");
            return true;
        }
    };
}


#endif
