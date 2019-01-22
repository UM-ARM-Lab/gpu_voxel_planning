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
        DenseGrid occupied_space;
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

        virtual std::vector<std::string> getLinkNames() const = 0;
        
        virtual std::vector<DenseGrid> getLinkOccupancies() = 0;

        DenseGrid getLinkOccupancy(const std::string &link_name)
        {
            DenseGrid g;
            const MetaPointCloud* clouds = robot_chain.getTransformedClouds();
            robot_chain.syncToHost();

            int16_t cloud_num = clouds->getCloudNumber(link_name);
            uint32_t cloud_size = clouds->getPointcloudSizes()[cloud_num];
            const gpu_voxels::Vector3f* cloud_ptr = clouds->getPointCloud(cloud_num);
            const std::vector<gpu_voxels::Vector3f> cloud(cloud_ptr, cloud_ptr + cloud_size);
            g.insertPointCloud(cloud, PROB_OCCUPIED);
            return g;
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

        bool operator==(const VictorArmConfig<joint_names> &other) const
        {
            if(joint_values.size() != other.joint_values.size())
            {
                return false;
            }
            
            for(size_t i=0; i<joint_values.size(); i++)
            {
                if(fabs(joint_values[i] - other.joint_values[i]) > 0.0001)
                {
                    return false;
                }
            }
            return true;
        }

        bool operator!=(const VictorArmConfig<joint_names> &other) const
        {
            return !(operator==(other));
        }
    };

    typedef VictorArmConfig<&right_arm_joint_names> VictorRightArmConfig;
    typedef VictorArmConfig<&left_arm_joint_names> VictorLeftArmConfig;
    



    
    class VictorRightArm: public Robot
    {
    public:
        const std::vector<std::string> collision_link_names{
            "victor_right_arm_link_2",
            "victor_right_arm_link_3",
                "victor_right_arm_link_4",
                "victor_right_arm_link_5",
                "victor_right_arm_link_6",
                "victor_right_arm_link_7",
                };

        const std::vector<std::string> gripper_link_names{
            "victor_right_gripper_palm",
                "victor_right_gripper_mounting_bracket",
                "victor_right_gripper_fingerA_base", "victor_right_gripper_fingerA_dist",
                "victor_right_gripper_fingerA_med", "victor_right_gripper_fingerA_prox",
                "victor_right_gripper_fingerB_base", "victor_right_gripper_fingerB_dist",
                "victor_right_gripper_fingerB_med", "victor_right_gripper_fingerB_prox",
                "victor_right_gripper_fingerC_base", "victor_right_gripper_fingerC_dist",
                "victor_right_gripper_fingerC_med", "victor_right_gripper_fingerC_prox"
                };
        

        VictorRightArm() :
            Robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_only.urdf")
            
        {
        }


        
        virtual std::vector<std::string> getLinkNames() const override
        {
            return std::vector<std::string> {
                "link_2, link_3, link_4, link_5, link_6, link_7, gripper"};
        }


        virtual std::vector<DenseGrid> getLinkOccupancies() override
        {
            std::vector<DenseGrid> link_occupancies;
            for(const std::string &link: collision_link_names)
            {
                link_occupancies.push_back(getLinkOccupancy(link));
            }
            for(const std::string &gripper_link: gripper_link_names)
            {
                DenseGrid tmp = getLinkOccupancy(gripper_link);
                link_occupancies.back().add(&tmp);
            }
            return link_occupancies;
        }
    };


















    
    class VictorLeftArmAndBase: public Robot
    {
    public:
        VictorLeftArmAndBase() :
            Robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_left_arm_and_body.urdf")
        {}

        virtual std::vector<std::string> getLinkNames() const override
        {
            throw std::logic_error("Not implemented for left arm");
        }

        virtual std::vector<DenseGrid> getLinkOccupancies() override
        {
            throw std::logic_error("Not implemented for left arm");
        }

    };
}



#endif
