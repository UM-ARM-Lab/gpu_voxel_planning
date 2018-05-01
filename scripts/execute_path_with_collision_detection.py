#! /usr/bin/env python

from openravepy import *
import or_victor.motion
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


ext_torque_limits = [5, 5, 5, 4, 2, 1, 1]

right_arm_listener = None
vm = None


def check_collision(motion_status_msg):
    global g_in_collision
    global g_links_in_contact
    with check_collision_lock:

        g_links_in_contact = []
        jt = vu.jvq_to_list(motion_status_msg.estimated_external_torque)

        col_indicies = [idx for idx in range(len(jt)) if abs(jt[idx]) > ext_torque_limits[idx]]

        g_in_collision = bool(col_indicies)

        if not g_in_collision:
            return False

        g_links_in_contact = ["victor_right_arm_link_" + str(idx+1) for idx in range(col_indicies[0], len(jt))]
        return True

def path_status_srv(req):
    global path_in_progress
    global last_collision_info
    rospy.loginfo("Path status srv")
    resp = AttemptPathResultResponse()
    with start_new_path_lock:
        resp.finished.data = not path_in_progress
        resp.ci = last_collision_info
    return resp
            
def attempt_path_srv(req):
    global ros_path
    global path_in_progress
    rospy.loginfo("Received new path attempt");

    resp = AttemptPathStartResponse()
    
    if(path_in_progress):
        rospy.loginfo("But there is already a path in progress");
        
        resp.started.data = False
        return resp
    
    resp.started.data = True

    rospy.loginfo("Triggering new path start");
    with start_new_path_lock:
        ros_path = [point.positions for point in req.path.points]
    rospy.loginfo("Returning");
    return resp


def start_attempt_path():
    global ros_path
    global path_in_progress
    global last_collision_info
    if ros_path:
        rospy.loginfo("Attempting path");
        path_in_progress = True
        with start_new_path_lock:
            last_collision_info = execute_path(ros_path)
        path_in_progress = False
        ros_path = []
        return


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
    
    rospy.loginfo("Executing path")
    col_links = None
    global in_collision
    in_collision = False
    
    def stop():
        global g_in_collision
        global g_links_in_contact
        global in_collision
        with check_collision_lock:
            if(g_in_collision):
                # rospy.loginfo("collision during motion")
                in_collision = True
                col_links = copy.deepcopy(g_links_in_contact)
                
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
    msg.collision_links = col_links


    if not in_collision:
        
        return msg
    voice.publish("collision")

    
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)

    next_poses = pu.densify(pu.travel_along(path, .2, cur_pos), .05)

    for pos in next_poses:
        jtp_msg = JointTrajectoryPoint()
        jtp_msg.positions = pos
        msg.collision_path.points.append(jtp_msg)
    
    backup_path = pu.travel_along(path, -0.2, cur_pos)

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
            voice.publish(g_links_in_contact[0][-1])
            rospy.sleep(1)




if __name__ == "__main__":
    
    rospy.init_node("execute_path_with_collision_detection")
    voice = rospy.Publisher("polly", String, queue_size=10)

    vm = or_victor.motion.MotionEnabledVictor(viewer=False)
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
    
    

    

    
    
