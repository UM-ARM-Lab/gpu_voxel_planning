#ifndef WORLDS_HPP
#define WORLDS_HPP


#include <arc_utilities/maybe.hpp>
#include "gpu_voxels_victor.hpp"
#include <sensor_msgs/JointState.h>


// Path densifyPath(const Path &path, int densify_factor);


class SimWorld
{
public:
    SimWorld();
    void initializeObstacles();
    void makeTable();
    void makeSlottedWall();
    bool executePath(const Path &path, size_t &last_index, bool add_col_set);

    bool attemptPath(const Path &path);

    void executeAndReturn(const Path &path);

    Maybe::Maybe<std::string> getCollisionLink(const VictorConfig &c);
    Maybe::Maybe<std::vector<std::string>> getCollisionLinks(const VictorConfig &c);

private:
    void initializeVictor();

public:    
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    GpuVoxelsVictor victor_model;
};

class RealWorld
{
public:
    RealWorld();
    ~RealWorld();
    bool attemptPath(const Path &path);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void spinUntilUpdate();
    void loadPointCloudFromFile();
    
public:
    gpu_voxels::GpuVoxelsSharedPtr gvl;
    GpuVoxelsVictor victor_model;
    ros::Subscriber joint_sub;
    ros::ServiceClient attempt_path_client;
    ros::ServiceClient get_attempt_status_client;

    bool update_victor_from_messages;
    bool pos_updated;
    int update_all_joint_count{0};
};



#endif
