#! /usr/bin/env python

from openravepy import *
import or_victor.motion
# import victor_hardware_interface.msg as vhimsg
import gpu_voxel_planning.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
from arc_utilities import ros_helpers as rh
from arc_utilities import path_utils as pu
from std_msgs.msg import String
import rospy
import IPython
from threading import Lock



pub = None
check_collision_lock = Lock()

in_collision = False
links_in_contact = []


ext_torque_limits = [5, 5, 5, 4, 2, 1, 1]

right_arm_listener = None


def check_collision(motion_status_msg):
    global in_collision
    global links_in_contact
    with check_collision_lock:

        links_in_contact = []
        jt = vu.jvq_to_list(motion_status_msg.estimated_external_torque)

        col_indicies = [idx for idx in range(len(jt)) if abs(jt[idx]) > ext_torque_limits[idx]]

        in_collision = bool(col_indicies)

        if not in_collision:
            return False

        links_in_contact = ["victor_right_arm_link_" + str(idx+1) for idx in range(col_indicies[0], len(jt))]
        return True
            



def execute_path(vm, path):
    """
    Executes a path on victor using the motion planner. 
    Torque limits will cause Victor to stop and back up

    Paramters:
    vm (motion.MotionEnabledVictor): victor motion
    path (or_victor.msg.Path): path to attempt to follow
    """
    global in_collision
    global links_in_contact

    
    def stop():
        global in_collision
        if(in_collision):
            rospy.loginfo("collision during motion")
        return in_collision
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

    if not in_collision:
        return True

    


    
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)
    backup_path = pu.travel_along(path, -0.2, cur_pos)

    rospy.loginfo("Collision detected. Backing up")

    vm.execute_trajectory(vm.traj_from_path(backup_path))
    
    # print("Checking torque limit")
    # print hit_torque_limit
    # if hit_torque_limit:
    #     # vm.plan_to_configuration(right_one, execute=True)
    #     print "Hit torque limit: Stopping"


    return False

def go_to(vm, config):
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)
    path = [cur_pos, config]
    return execute_path(vm, path)



def speak_collision_link():
    while(not rospy.is_shutdown()):

        with check_collision_lock:
            play_msg = in_collision

        if play_msg:
            pub.publish(links_in_contact[0][-1])
            rospy.sleep(1)




if __name__ == "__main__":
    rospy.init_node("execute_path_with_collision_detection")

    pub = rospy.Publisher("polly", String, queue_size=10)

    vm = or_victor.motion.MotionEnabledVictor(viewer=False)
    vm.set_manipulator("right_arm")
    vm.change_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=vu.Stiffness.MEDIUM)

    sub = rospy.Subscriber("right_arm/motion_status", MotionStatus, check_collision)
    right_arm_listener = rh.Listener("right_arm/motion_status", MotionStatus)

    while(True):
        while not go_to(vm, [-.25, .5, -.18, -1.2, .4, .4, -.7]):
            rospy.sleep(1)
        while not go_to(vm, [-.25, -.11, -.18, -1.0, .4, .4, -.7]):
            rospy.sleep(1)


    # speak_collision_link()
    print("Planning right arm to configuration")
    
    

    

    
    
