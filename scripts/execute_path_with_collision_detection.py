#! /usr/bin/env python

from openravepy import *
import arm_or_robots.motion_victor
# import victor_hardware_interface.msg as vhimsg
import gpu_voxel_planning.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
from arc_utilities import ros_helpers as rh
from arc_utilities import path_utils as pu
from gpu_voxel_planning.msg import CollisionInformation
from gpu_voxel_planning.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import IPython
from threading import Lock
import copy



voice = None
check_collision_lock = Lock()
start_new_path_lock = Lock()

g_in_collision = False
g_links_in_contact = []
last_collision_info = CollisionInformation()
ros_path = []
path_in_progress = False


ext_torque_limits = [20, 20, 15, 5, 4, 3, .9]

right_arm_listener = None
vm = None


def check_collision(motion_status_msg):
    global g_in_collision
    global g_links_in_contact
    with check_collision_lock:

        g_links_in_contact = []
        jt = vu.jvq_to_list(motion_status_msg.estimated_external_torque)

        col_indicies = [idx for idx in range(len(jt)) if abs(jt[idx]) > ext_torque_limits[idx]]

        if(col_indicies):
            print jt
            print(col_indicies)
        
        g_in_collision = bool(col_indicies)

        if not g_in_collision:
            return False

        g_links_in_contact = ["victor_right_arm_link_" + str(idx+1) for idx in range(col_indicies[-1], len(jt))]
        print("check_collision links in contact")
        print(g_links_in_contact)
        return True

def path_status_srv(req):
    global path_in_progress
    global last_collision_info
    # rospy.loginfo("Path status srv")
    resp = AttemptPathResultResponse()
    with start_new_path_lock:
        resp.finished = not path_in_progress
        resp.ci = last_collision_info
    return resp
            
def attempt_path_srv(req):
    global ros_path
    global path_in_progress
    rospy.loginfo("Received new path attempt");

    resp = AttemptPathStartResponse()
    
    if(path_in_progress):
        rospy.loginfo("But there is already a path in progress");
        
        resp.started = False
        return resp

    resp.started = True

    rospy.loginfo("Triggering new path start");
    with start_new_path_lock:
        path_in_progress = True
        ros_path = [point.positions for point in req.path.points]
    rospy.loginfo("Returning");
    return resp


def start_attempt_path():
    global ros_path
    global path_in_progress
    global last_collision_info

    path_to_execute = None
    with start_new_path_lock:
        if not ros_path:
            return
        path_to_execute = copy.deepcopy(ros_path)
        
        
    rospy.loginfo("Attempting path");
    ci = execute_path(ros_path)
    with start_new_path_lock:
        last_collision_info = ci
        ros_path = []
        path_in_progress = False



def execute_path(path):
    """
    Executes a path on victor using the motion planner. 
    Torque limits will cause Victor to stop and back up

    Paramters:
    vm (motion.MotionEnabledVictor): victor motion
    path (or_victor.msg.Path): path to attempt to follow

    Returns:
    collision information msg
    """
    global g_in_collision
    global g_links_in_contact
    global vm
    global path_in_progress
    global col_links
    
    rospy.loginfo("Executing path")
    col_links = []
    global in_collision
    in_collision = False
    
    def stop():
        global g_in_collision
        global g_links_in_contact
        global in_collision
        global col_links
        with check_collision_lock:
            if(g_in_collision):
                # rospy.loginfo("collision during motion")
                in_collision = True
                col_links = copy.deepcopy(g_links_in_contact)

                print("stop links in contact")
                print(g_links_in_contact)
                print(col_links)
            return g_in_collision
        # return 
        # global hit_torque_limit
        # tqs = vm.active_arm_motion_status().estimated_external_torque
        # for tq in vu.jvq_to_list(tqs):
        #     if abs(tq) > 4:
        #         print("Torque limit exceeded")
        #         hit_torque_limit = True
        #         return True
        # return False

    vm.action_terminate_check_callback = stop

    traj = vm.traj_from_path(path)
    vm.execute_trajectory(traj)

    vm.action_terminate_check_callback = None

    msg = CollisionInformation()
    msg.collided = in_collision


    if not in_collision:

        # Check for collision at end of trajectory
        rospy.sleep(0.2)
        stop()
        if in_collision:
            print("This extra check did something!")

    if not in_collision:
        return msg

    # speak_collision_link()
    
    
    print("assignment links in contact")
    print(col_links)
    msg.collision_links = col_links
    
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)

    next_poses = pu.densify(pu.travel_along(path, .15, cur_pos), .03)

    for pos in next_poses:
        jtp_msg = JointTrajectoryPoint()
        jtp_msg.positions = pos
        msg.collision_path.points.append(jtp_msg)
    
    backup_path = pu.travel_along(path, -0.1, cur_pos)

    rospy.loginfo("Collision detected. Backing up")

    vm.execute_trajectory(vm.traj_from_path(backup_path))
    return msg

def go_to(config):
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)
    path = [cur_pos, config]
    return execute_path(path)



def speak_collision_link():
    while(not rospy.is_shutdown()):

        with check_collision_lock:
            play_msg = g_in_collision

            if play_msg:
                voice.publish("collision link " + g_links_in_contact[0][-1])
                rospy.sleep(1)




if __name__ == "__main__":
    
    rospy.init_node("execute_path_with_collision_detection")
    voice = rospy.Publisher("polly", String, queue_size=10)

    vm = arm_or_robots.motion_victor.MotionEnabledVictor(viewer=False, world_frame="gpu_voxel_world")
    vm.set_manipulator("right_arm")
    vm.change_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=vu.Stiffness.MEDIUM)

    sub = rospy.Subscriber("right_arm/motion_status", MotionStatus, check_collision)
    s = rospy.Service("attempt_path_on_victor", AttemptPathStart, attempt_path_srv)
    s = rospy.Service("get_path_status", AttemptPathResult, path_status_srv)
    right_arm_listener = rh.Listener("right_arm/motion_status", MotionStatus)

    # while(True):
    #     while go_to([-.25, .5, -.18, -1.2, .4, .4, -.7]).collided:
    #         rospy.sleep(1)
    #     while go_to([-.25, -.11, -.18, -1.0, .4, .4, -.7]).collided:
    #         rospy.sleep(1)
    #     while go_to([-.25, -.11, .3, -1.0, .4, .4, -.7]).collided:
    #         rospy.sleep(1)

    rospy.loginfo("Waiting for start path messages")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        start_attempt_path()

    # speak_collision_link()
    print("Planning right arm to configuration")
    
    

    

    
    
