#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP


#include "state.hpp"
#include "gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class Scenario
    {
    public:
        VictorRightArm victor;
        robot::JointValueMap goal_config;
        virtual State& getState() = 0;
        virtual const State& getState() const = 0;
        virtual std::string getName() const = 0;
        
        virtual bool completed() const
        {
            return VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal_config);
        }

        virtual void viz(const GpuVoxelRvizVisualizer& viz)
        {
        }

        Scenario(){}
    };


    
    class SimulationScenario : public Scenario
    {
    public:
        SimulationState s;
        DenseGrid true_world;

        SimulationScenario() : s(victor){}
        // virtual DenseGrid& getTrueObstacles() = 0;
        // virtual const DenseGrid& getTrueObstacles() const = 0;
        // virtual SimulationState& getSimulationState() = 0;
        // virtual const SimulationState& getSimulationState() const = 0;

        virtual void validate()
        {
            s.robot.set(s.getCurConfig().asMap());
            if(s.robot.occupied_space.overlapsWith(&true_world))
            {
                std::cerr << "Start configuration overlaps with obstacle\n";
                throw(std::invalid_argument("Start configuration is invalid\n"));
            }
            s.robot.set(goal_config);
            if(s.robot.occupied_space.overlapsWith(&true_world))
            {
                std::cerr << "Goal configuration overlaps with obstacle\n";
                throw(std::invalid_argument("Goal configuration is invalid\n"));
            }
        }

        virtual void viz(const GpuVoxelRvizVisualizer& viz) override
        {
            viz.grid_pub.publish(visualizeDenseGrid(getTrueObstacles(), viz.global_frame,
                                                    "true_obstacles", makeColor(0.5, 0.5, 0.5, 0.5)));

            viz.grid_pub.publish(visualizeDenseGrid(s.known_obstacles, viz.global_frame,
                                                    "known_obstacles", makeColor(0,0,0,1)));

            std_msgs::ColorRGBA robot_color = makeColor(0.7, 0.5, 0.4, 1.0);
            viz.grid_pub.publish(visualizeDenseGrid(s.robot_self_collide_obstacles, viz.global_frame,
                                                    "passive_robot", robot_color));
            viz.grid_pub.publish(visualizeDenseGrid(s.robot.occupied_space, viz.global_frame,
                                                    "active_robot", robot_color));
            viz.vizChs(s.chs);
        }


        virtual DenseGrid& getTrueObstacles()
        {
            return true_world;
        }

        virtual const DenseGrid& getTrueObstacles() const
        {
            return true_world;
        }

        virtual SimulationState& getSimulationState()
        {
            return s;
        }

        virtual const SimulationState& getSimulationState() const
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
        const std::string name;


        TableWithBox(bool table_known=true, bool visible_cave_known=false, bool full_cave_known=false) :
            name(std::string("Table_with_Box_") +
                 "table_" + (table_known ? "" : "un") + "known_" + 
                 "visible_cave_" + (visible_cave_known ? "" : "un" ) + "known_" + 
                 "full_cave_" + (full_cave_known ? "" : "un" ) + "known"
                )
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


        virtual std::string getName() const override
        {
            return name;
        }



        void addTable(DenseGrid &g)
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
            cavetopd = Vector3f(0.3, 0.5, 0.033);
            cavesidedim = Vector3f(0.033, cavetopd.y, caveheight.z);
            cavesideoffset = Vector3f(cavetopd.x, 0.0, 0.0);

        }

        void addVisibleCave(DenseGrid &g)
        {
            setCaveDims();
            g.insertBox(cavecorner+caveheight, cavecorner+caveheight+cavetopd+Vector3f(0.033, 0, 0)); //top
            g.insertBox(cavecorner, cavecorner+cavesidedim);
            
        }

        void addCave(DenseGrid &g)
        {
            setCaveDims();
            addVisibleCave(g);

            g.insertBox(cavecorner+cavesideoffset,
                        cavecorner+cavesideoffset+cavesidedim);
        }
    };



    /****************************************
     **         SlottedWall
     ****************************************/
    class SlottedWall : public SimulationScenario
    {
        const std::string name;
    public:
        SlottedWall(bool all_known):
            name(std::string("Sloted Wall") +
                 "obstacles" + (all_known ? "" : "un") + "known")
        {
            addLeftArm();
            addSlottedWall(true_world);
            if(all_known)
            {
                addSlottedWall(s.known_obstacles);
            }
            // s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
            // goal_config = VictorRightArmConfig(std::vector<double>{0, 0.32, 0, -1.32, -0.2, 0.9, 0.3}).asMap();


            s.current_config = VictorRightArmConfig(std::vector<double>{-1.5, 1.0, -0.5, -0.5,0,0,0}).asMap();
            goal_config = VictorRightArmConfig(std::vector<double>{0, 0.72, -0.3, -1.32, -1.2, 0.9, 0.3}).asMap();

            victor.set(s.current_config);
        }

        std::string getName() const
        {
            return "SlottedWall";
        }

        void addSlottedWall(DenseGrid &g)
        {
            double lower_wall_height = 1.1;
            double gap_height = .4;

            Vector3f lfwc(1.5, 1.6, 0.0); //lower front wall corner
            Vector3f lfwd(0.04, 0.4, lower_wall_height);
    
            Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height); //upper front wall
            Vector3f ufwd(0.04, 0.4, 0.3);
    
            Vector3f mfwc(1.5, 1.8, 0);  //middle front wall
            Vector3f mfwd(0.04, 0.2, 1.5);
   
            Vector3f lswc = lfwc;  // lower side wall corner
            Vector3f lswd(.75, 0.04, lower_wall_height); //lower side wall dims
            Vector3f cswc = ufwc; //close side wall corner
            Vector3f cswd(0.2, 0.04, 0.3);
            Vector3f fswc(1.95, 1.6, lower_wall_height); //far side wall corner
            Vector3f fswd(0.3, 0.04, 0.6);
            Vector3f mswc(1.95, 1.6, lower_wall_height+gap_height+.1); //far side wall corner
            Vector3f mswd(0.3, 0.04, 0.2);

            g.insertBox(lfwc, lfwc+lfwd);
            g.insertBox(ufwc, ufwc+ufwd);
            g.insertBox(mfwc, mfwc+mfwd);
            
            g.insertBox(lswc, lswc+lswd);
            g.insertBox(cswc, cswc+cswd);
            g.insertBox(fswc, fswc+fswd);
            g.insertBox(mswc, mswc+mswd);
        }
    };











    /****************************************
     **         Bookshelf
     ****************************************/
    class Bookshelf : public SimulationScenario
    {
        const std::string name;
    public:
        Bookshelf(bool all_known):
            name(std::string("Bookshelf") +
                 "obstacles" + (all_known ? "" : "un") + "known")
        {
            addLeftArm();
            addBookshelf(true_world);
            addTable(true_world);

            robot::JointValueMap jvm;
            jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
            jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
            jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
            victor.set(jvm);

            if(all_known)
            {
                addBookshelf(s.known_obstacles);
                addTable(s.known_obstacles);
            }
            // s.current_config = VictorRightArmConfig(std::vector<double>
            //                                         {-0.9, 1.3, -0.3, -0.8, 0.0, 0.2, 0.3}).asMap();
            s.current_config = VictorRightArmConfig(std::vector<double>
                                                    {-1.2, 1.3, -0.8, 0.4, 0.4, 0.3, 0.3}).asMap();
            goal_config = VictorRightArmConfig(std::vector<double>
                                               {0.3, 1.2, -0.3, 1.5, 0, -0.7, -0.9}).asMap();

            victor.set(s.current_config);
        }

        std::string getName() const
        {
            return "Bookshelf";
        }

        void addBookshelf(DenseGrid &g)
        {
            double lower_wall_height = 1.1;
            double gap_height = .4;

            Vector3f lfwc(1.5, 1.6, 0.0); //lower front wall corner
            Vector3f lfwd(0.04, 0.4, lower_wall_height);
    
            Vector3f ufwc(1.5, 1.6, lower_wall_height + gap_height); //upper front wall
            Vector3f ufwd(0.04, 0.4, 0.3);
    
            Vector3f mfwc(1.5, 1.8, 0);  //middle front wall
            Vector3f mfwd(0.04, 0.2, 1.5);
   
            Vector3f lswc = lfwc;  // lower side wall corner
            Vector3f lswd(.75, 0.04, lower_wall_height); //lower side wall dims
            Vector3f cswc = ufwc; //close side wall corner
            Vector3f cswd(0.2, 0.04, 0.3);
            Vector3f fswc(1.95, 1.6, lower_wall_height); //far side wall corner
            Vector3f fswd(0.3, 0.04, 0.6);
            Vector3f mswc(1.95, 1.6, lower_wall_height+gap_height+.1); //far side wall corner
            Vector3f mswd(0.3, 0.04, 0.2);


            double bookshelf_width = 0.8;
            double bookshelf_height = 1.6;
            double bookshelf_depth = 0.4;
            Vector3f backwallc(1.2, 0.8, 0.0); //backwall cornder
            Vector3f backwall_thickness(bookshelf_width, 0.04, bookshelf_height);
            Vector3f sidewall(0.04, bookshelf_depth,  bookshelf_height);
            Vector3f swoff(bookshelf_width, 0, 0);
            Vector3f shelf(bookshelf_width + 0.04, bookshelf_depth, 0.02);
            Vector3f shelf_spacing(0, 0, 0.4);

            Vector3f bookc(1.6, 0.82, 1.2);
            Vector3f bookd(0.05, 0.3, 0.3);

            g.insertBox(backwallc, backwallc + backwall_thickness);
            g.insertBox(backwallc, backwallc + sidewall);
            g.insertBox(backwallc + swoff, backwallc + sidewall + swoff);
            for(int i=0; i<5; i++)
            {
                g.insertBox(backwallc + shelf_spacing*(float)i, backwallc + shelf + shelf_spacing*(float)i);
            }
            g.insertBox(bookc, bookc+bookd);


            // g.insertBox(lfwc, lfwc+lfwd);
            // g.insertBox(ufwc, ufwc+ufwd);
            // g.insertBox(mfwc, mfwc+mfwd);
            
            // g.insertBox(lswc, lswc+lswd);
            // g.insertBox(cswc, cswc+cswd);
            // g.insertBox(fswc, fswc+fswd);
            // g.insertBox(mswc, mswc+mswd);
        }

        void addTable(DenseGrid &g)
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

    };

}


#endif
