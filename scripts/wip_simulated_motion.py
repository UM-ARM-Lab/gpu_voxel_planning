#! /usr/bin/env python

from openravepy import *
import or_victor.motion
# import victor_hardware_interface.msg as vhimsg
import gpu_voxel_planning.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
import rospy
import IPython
from arc_utilities import path_utils as pu

import time



# right_one = [-1.69, 1.0, 0.0, -0.5, -0.5, 0.5, 0.0]
# right_two = [0, 1.0, 0.0, -0.5, -0.5, 0.5, 0.0]
right_two = [-.02, 1.0, 0.0, -0.5, 0, 0.9, -0.4]

hit_torque_limit = False


def execute_path(vm, path):
    """
    Executes a path on victor using the motion planner. 
    Torque limits will cause Victor to stop and back up

    Paramters:
    vm (motion.MotionEnabledVictor): victor motion
    path (or_victor.msg.Path): path to attempt to follow
    """
    global hit_torque_limit

    hit_torque_limit = False
    
    def stop():
        global hit_torque_limit
        tqs = vm.active_arm_motion_status().estimated_external_torque;
        for tq in vu.jvq_to_list(tqs):
            if abs(tq) > 4:
                print("Torque limit exceeded")
                hit_torque_limit = True
                return True
        return False

    vm.action_terminate_check_callback = stop

    traj = vm.traj_from_path_msg(plan_resp.path)
    vm.execute_trajectory(traj)

    vm.action_terminate_check_callback = None


    if hit_torque_limit:

        print "Hit torque limit: Backing off"
        # IPython.embed()
        cur = vu.jvq_to_list(vm.active_arm_motion_status().measured_joint_position)
        # IPython.embed()
        path = [p.positions for p in plan_resp.path.points]
        backup_path = pu.travel_along(path, -0.3, cur)
        vm.execute_trajectory(vm.traj_from_path(backup_path))


        


    return (not hit_torque_limit)

def wiggle(vm):
    cur = vu.jvq_to_list(vm.active_arm_motion_status.measured_joint_position)
    
    delta = 0.1
    
    new = cur
    for ind in range(7):
        new[ind] = cur[ind] + delta

        

if __name__ == "__main__":
    rospy.init_node("basic_motion")



    vm = or_victor.motion.MotionEnabledVictor()
    vm.set_manipulator("right_arm")
    vm.change_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=vu.Stiffness.MEDIUM)


    rospy.wait_for_service("plan_path")
    gpu_path = rospy.ServiceProxy("plan_path", gvpsrv.PlanPath)
    
    req = gvpsrv.PlanPathRequest()
    # req.start = vu.list_to_jvq(right_one)
    req.goal = vu.list_to_jvq(right_two)

    success = False
    while success is False:
        try:
            print("Planning path")
            # IPython.embed()
            req.start = vm.active_arm_motion_status().measured_joint_position
            plan_resp = gpu_path(req)

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            assert False

        print("Executing path")
        success = execute_path(vm, plan_resp.path)


    print "Goal reached! Finished for now."
    IPython.embed()
    

    # if success:
    #     req = gvpsrv.PlanPathRequest()
    #     req.start = vu.list_to_jvq(right_two)
    #     req.goal = vu.list_to_jvq(right_one)

    
    #     try:
    #         plan_resp = gpu_path(req)
    #     except rospy.ServiceException as exc:
    #         print("Service did not process request: " + str(exc))


    #     execute_path(vm, plan_resp.path)
    # else:
    #     pass

    
    
    # print("Finished")
    # print("")
    
    
