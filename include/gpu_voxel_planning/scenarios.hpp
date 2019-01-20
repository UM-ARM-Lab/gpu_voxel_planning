#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP


#include "state.hpp"

namespace GVP
{
    class Scenario
    {
    public:
        virtual ProbGrid& getTrueObstacles() = 0;
        virtual State& getState() = 0;
        virtual const ProbGrid& getTrueObstacles() const = 0;
        virtual const State& getState() const = 0;
    };




    class TableWithBox : public Scenario
    {
    public:
        State s;
        ProbGrid true_world;
        VictorRightArm victor;
        robot::JointValueMap goal_config;



        TableWithBox(bool table_known=true, bool cave_known=false): s(victor)
        {
            addLeftArm();
            addTable(true_world);
            addCave(true_world);

            if(table_known)
            {
                addTable(s.known_obstacles);
            }

            if(cave_known)
            {
                addCave(s.known_obstacles);
            }

            s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
            goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();

        }

        virtual ProbGrid& getTrueObstacles() override
        {
            return true_world;
        }

        virtual const ProbGrid& getTrueObstacles() const override
        {
            return true_world;
        }

        virtual State& getState() override
        {
            return s;
        }

        virtual const State& getState() const override
        {
            return s;
        }
        

        void addLeftArm()
        {
            VictorLeftArmAndBase left;
            left.set(VictorLeftArmConfig(std::vector<double>{1.57, 1.57, 0, 0, 0, 0 ,0}).asMap());
            s.robot_self_collide_obstacles.add(&left.occupied_space);

            robot::JointValueMap jvm;
            jvm["victor_right_gripper_fingerA_joint_2"] = 1.5;
            jvm["victor_right_gripper_fingerB_joint_2"] = 1.5;
            jvm["victor_right_gripper_fingerC_joint_2"] = 1.5;
            victor.set(jvm);
        }

        void addTable(ProbGrid &g)
        {
            Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
            Vector3f tc(1.7, 1.4, 0.9); //table corner
            Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
            Vector3f tld(.033, 0.033, tc.z); //table leg dims


            g.insertBox(tc, tc+td);
            g.insertBox(tcf, tcf+tld);
            g.insertBox(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld);
            g.insertBox(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld);
            g.insertBox(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                        Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld);

        }

        void addCave(ProbGrid &g)
        {
            Vector3f cavecorner = Vector3f(1.7, 2.0, 0.9);
            Vector3f caveheight(0.0, 0.0, 0.4);
            Vector3f cavetopd(0.4, 0.5, 0.033);
            Vector3f cavesidedim(0.033, cavetopd.y, caveheight.z);
            Vector3f cavesideoffset(cavetopd.x, 0.0, 0.0);
            Vector3f caveholecorner = cavecorner + cavesideoffset + Vector3f(0, 0.15, 0.15);
            Vector3f caveholesize(.04, .15, .15);

            g.insertBox(cavecorner+caveheight, cavecorner+caveheight+cavetopd+Vector3f(0.033, 0, 0)); //top

            g.insertBox(cavecorner, cavecorner+cavesidedim);

            g.insertBox(cavecorner+cavesideoffset,
                        cavecorner+cavesideoffset+cavesidedim);

        }
    };
}


#endif
