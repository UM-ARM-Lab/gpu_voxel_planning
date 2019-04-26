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
        ObstacleConfiguration true_obstacles;

        SimulationScenario() : s(victor)
        {
        }
        // virtual DenseGrid& getTrueObstacles() = 0;
        // virtual const DenseGrid& getTrueObstacles() const = 0;
        // virtual SimulationState& getSimulationState() = 0;
        // virtual const SimulationState& getSimulationState() const = 0;

        virtual void validate()
        {
            s.robot.set(s.getCurConfig().asMap());
            if(s.robot.occupied_space.overlapsWith(&true_obstacles.occupied))
            {
                std::cerr << "Start configuration overlaps with obstacle\n";
                throw(std::invalid_argument("Start configuration is invalid\n"));
            }
            s.robot.set(goal_config);
            if(s.robot.occupied_space.overlapsWith(&true_obstacles.occupied))
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
            s.bel->viz(viz);
        }


        virtual DenseGrid& getTrueObstacles()
        {
            return true_obstacles.occupied;
        }

        virtual const DenseGrid& getTrueObstacles() const
        {
            return true_obstacles.occupied;
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
            true_obstacles.add(getTable());
            true_obstacles.add(getVisibleCave());
            true_obstacles.add(getCaveBack());

            if(table_known)
            {
                Object table = getTable();
                s.known_obstacles.add(&table.occupied);
            }

            if(visible_cave_known)
            {
                Object known_cave = getVisibleCave();
                s.known_obstacles.add(&known_cave.occupied);
            }

            if(full_cave_known)
            {
                Object cave_back = getCaveBack();
                s.known_obstacles.add(&cave_back.occupied);
            }

            s.current_config = VictorRightArmConfig(std::vector<double>{0,0,0,0,0,0,0}).asMap();
            goal_config = VictorRightArmConfig(std::vector<double>{-0.15, 1.0, 0, -0.5, 0, 1.0, 0}).asMap();
        }


        virtual std::string getName() const override
        {
            return name;
        }



        Object getTable()
        {
            Object table;
            Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
            Vector3f tc(1.7, 1.4, 0.9); //table corner
            Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
            Vector3f tld(.033, 0.033, tc.z); //table leg dims


            table.add(AABB(tc, tc+td));
            table.add(AABB(tcf, tcf+tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
            return table;
        }

        void setCaveDims()
        {
            cavecorner = Vector3f(1.7, 2.0, 0.9);
            caveheight = Vector3f(0.0, 0.0, 0.4);
            cavetopd = Vector3f(0.3, 0.5, 0.033);
            cavesidedim = Vector3f(0.033, cavetopd.y, caveheight.z);
            cavesideoffset = Vector3f(cavetopd.x, 0.0, 0.0);

        }

        Object getVisibleCave()
        {
            setCaveDims();
            Object cave;
            cave.add(AABB(cavecorner+caveheight, cavecorner+caveheight+cavetopd+Vector3f(0.033, 0, 0))); //top
            cave.add(AABB(cavecorner, cavecorner+cavesidedim));
            return cave;
        }

        Object getCaveBack()
        {
            setCaveDims();
            Object cave_back;
            cave_back.add(AABB(cavecorner+cavesideoffset,
                               cavecorner+cavesideoffset+cavesidedim));
            return cave_back;
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
            true_obstacles.add(getSlottedWall());
            if(all_known)
            {
                Object slotted_wall = getSlottedWall();
                s.known_obstacles.add(&slotted_wall.occupied);
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

        Object getSlottedWall()
        {
            Object slotted_wall;
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

            slotted_wall.add(AABB(lfwc, lfwc+lfwd));
            slotted_wall.add(AABB(ufwc, ufwc+ufwd));
            slotted_wall.add(AABB(mfwc, mfwc+mfwd));
            
            slotted_wall.add(AABB(lswc, lswc+lswd));
            slotted_wall.add(AABB(cswc, cswc+cswd));
            slotted_wall.add(AABB(fswc, fswc+fswd));
            slotted_wall.add(AABB(mswc, mswc+mswd));
            return slotted_wall;
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
            true_obstacles.add(getBookshelf());
            true_obstacles.add(getTable());

            robot::JointValueMap jvm;
            jvm["victor_right_gripper_fingerA_joint_2"] = 0.0;
            jvm["victor_right_gripper_fingerB_joint_2"] = 0.0;
            jvm["victor_right_gripper_fingerC_joint_2"] = 0.0;
            victor.set(jvm);

            if(all_known)
            {
                Object bookshelf = getBookshelf();
                s.known_obstacles.add(&bookshelf.occupied);
                Object table = getTable();
                s.known_obstacles.add(&table.occupied);
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

        Object getBookshelf()
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

            Object bookshelf;

            bookshelf.add(AABB(backwallc, backwallc + backwall_thickness));
            bookshelf.add(AABB(backwallc, backwallc + sidewall));
            bookshelf.add(AABB(backwallc + swoff, backwallc + sidewall + swoff));
            for(int i=0; i<5; i++)
            {
                bookshelf.add(AABB(backwallc + shelf_spacing*(float)i, backwallc + shelf + shelf_spacing*(float)i));
            }
            bookshelf.add(AABB(bookc, bookc+bookd));

            return bookshelf;
            // g.insertBox(lfwc, lfwc+lfwd);
            // g.insertBox(ufwc, ufwc+ufwd);
            // g.insertBox(mfwc, mfwc+mfwd);
            
            // g.insertBox(lswc, lswc+lswd);
            // g.insertBox(cswc, cswc+cswd);
            // g.insertBox(fswc, fswc+fswd);
            // g.insertBox(mswc, mswc+mswd);
        }

        Object getTable()
        {
            Vector3f td(30.0 * 0.0254, 42.0 * 0.0254, 1.0 * 0.0254); //table dimensions
            Vector3f tc(1.7, 1.4, 0.9); //table corner
            Vector3f tcf(1.7, 1.4, 0.0); //table corner at floor
            Vector3f tld(.033, 0.033, tc.z); //table leg dims
            
            Object table;
            table.add(AABB(tc, tc+td));
            table.add(AABB(tcf, tcf+tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, 0, 0) + tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(0, td.y-tld.y, 0) + tld));
            table.add(AABB(Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0),
                           Vector3f(tc.x, tc.y, 0) + Vector3f(td.x-tld.x, td.y-tld.y, 0) + tld));
            return table;

        }

    };

}


#endif
