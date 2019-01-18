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
    };



    


    class VictorRightArmConfig
    {
    public:
        std::vector<double> joint_values;

        VictorRightArmConfig(std::vector<double> joint_values) : joint_values(joint_values){};

        VictorRightArmConfig(const double* values) {
            for(size_t i=0; i<right_arm_joint_names.size(); i++)
            {
                joint_values.push_back(values[i]);
            }
        }

        VictorRightArmConfig(const robot::JointValueMap &jvm)
        {
            for(size_t i=0; i<right_arm_joint_names.size(); i++)
            {
                joint_values.push_back(jvm[right_arm_joint_names[i]]);
            }
        }

        robot::JointValueMap asMap() const
        {
            robot::JointValueMap jvm;
            for(size_t i=0; i<right_arm_joint_names.size(); i++)
            {
                jvm[right_arm_joint_names[i]] = joint_values[i];
            }
            return jvm;
        }

        std::vector<double> asVector() const
        {
            return joint_values;
        }
    };



    
    class Victor : public Robot
    {
    public:
        Victor(const std::string &path_to_urdf_file) : Robot(path_to_urdf_file)
        {
        }

        void set(const VictorRightArmConfig &config)
        {
            robot_chain.setConfiguration(config.asMap());
            occupied_space.insertMetaPointCloud(*robot_chain.getTransformedClouds(), PROB_OCCUPIED);
        }
    };
}



#endif
