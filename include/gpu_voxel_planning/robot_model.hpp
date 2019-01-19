#ifndef GPU_PLANNING_ROBOT_MODEL_HPP
#define GPU_PLANNING_ROBOT_MODEL_HPP

#include "prob_map.hpp"
#include "common_names.hpp"
#include "hacky_functions.hpp"

namespace GVP
{
    class Robot
    {
    public:
        ProbGrid occupied_space;
        robot::UrdfRobot robot_chain;

        Robot(const std::string &path_to_urdf_file) :
            robot_chain(VOXEL_SIDE_LENGTH, path_to_urdf_file, false)
        {
            robot::JointValueMap jvm;
            robot_chain.setConfiguration(jvm);
            occupied_space.insertMetaPointCloud(*robot_chain.getTransformedClouds(), PROB_OCCUPIED);
        }

        void set(const robot::JointValueMap &map)
        {
            robot_chain.setConfiguration(map);
            occupied_space.clearMap();
            occupied_space.insertMetaPointCloud(*robot_chain.getTransformedClouds(), PROB_OCCUPIED);
        }

        virtual bool isValid()
        {
            return true;
        }
    };



    

    template<const std::vector<std::string>* joint_names>
    class VictorArmConfig
    {
    public:
        std::vector<double> joint_values;

        VictorArmConfig() {};

        VictorArmConfig(std::vector<double> joint_values) : joint_values(joint_values){};

        VictorArmConfig(const double* values) {
            for(size_t i=0; i<joint_names->size(); i++)
            {
                joint_values.push_back(values[i]);
            }
        }

        VictorArmConfig(const robot::JointValueMap &jvm)
        {
            for(size_t i=0; i<joint_names->size(); i++)
            {
                joint_values.push_back(jvm.at(joint_names->at(i)));
            }
        }

        robot::JointValueMap asMap() const
        {
            robot::JointValueMap jvm;
            for(size_t i=0; i<joint_names->size(); i++)
            {
                jvm[joint_names->at(i)] = joint_values[i];
            }
            return jvm;
        }

        std::vector<double> asVector() const
        {
            return joint_values;
        }
    };

    typedef VictorArmConfig<&right_arm_joint_names> VictorRightArmConfig;
    typedef VictorArmConfig<&left_arm_joint_names> VictorLeftArmConfig;



    
    class VictorRightArm: public Robot
    {
    public:
        VictorRightArm() :
            Robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_only.urdf")
            
        {
        }
    };

    class VictorLeftArmAndBase: public Robot
    {
    public:
        VictorLeftArmAndBase() :
            Robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_left_arm_and_body.urdf")
        {}
    };
}



#endif
