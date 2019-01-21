#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP


#include "state.hpp"

namespace GVP
{
    class Scenario
    {
    public:
        VictorRightArm victor;
        robot::JointValueMap goal_config;
        
        virtual State& getState() = 0;
        virtual const State& getState() const = 0;
        
        virtual bool completed() const
        {
            return VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal_config);
        }

        Scenario(){}
    };


    
    class SimulationScenario : public Scenario
    {
    public:
        SimulationState s;
        ProbGrid true_world;
        SimulationScenario() : s(victor){}
        virtual ProbGrid& getTrueObstacles() = 0;
        virtual const ProbGrid& getTrueObstacles() const = 0;
        virtual SimulationState& getSimulationState() = 0;
        virtual const SimulationState& getSimulationState() const = 0;
        
    };



    /****************************************
     **         Table With Box
     ****************************************/
    class TableWithBox : public SimulationScenario
    {
    public:
        Vector3f cavecorner;
        Vector3f caveheight;
        Vector3f cavetopd;
        Vector3f cavesidedim;
        Vector3f cavesideoffset;


        TableWithBox(bool table_known=true, bool visible_cave_known=false, bool full_cave_known=false)
        {
            addLeftArm();
            addTable(true_world);
            addCave(true_world);

            if(table_known)
            {
                addTable(s.known_obstacles);
            }

            if(visible_cave_known)
            {
                addVisibleCave(s.known_obstacles);
            }

            if(full_cave_known)
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

        virtual SimulationState& getSimulationState() override
        {
            return s;
        }

        virtual const SimulationState& getSimulationState() const override
        {
            return s;
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

        void setCaveDims()
        {
            cavecorner = Vector3f(1.7, 2.0, 0.9);
            caveheight = Vector3f(0.0, 0.0, 0.4);
            cavetopd = Vector3f(0.4, 0.5, 0.033);
            cavesidedim = Vector3f(0.033, cavetopd.y, caveheight.z);
            cavesideoffset = Vector3f(cavetopd.x, 0.0, 0.0);

        }

        void addVisibleCave(ProbGrid &g)
        {
            setCaveDims();
            g.insertBox(cavecorner+caveheight, cavecorner+caveheight+cavetopd+Vector3f(0.033, 0, 0)); //top
            g.insertBox(cavecorner, cavecorner+cavesidedim);
            
        }

        void addCave(ProbGrid &g)
        {
            setCaveDims();
            addVisibleCave(g);

            g.insertBox(cavecorner+cavesideoffset,
                        cavecorner+cavesideoffset+cavesidedim);



        }
    };
}


#endif
