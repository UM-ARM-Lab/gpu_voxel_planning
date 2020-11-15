#! /usr/bin/env python

from openravepy import *
import or_victor.motion
# import victor_hardware_interface_msgs.msg as vhimsg
import gpu_voxel_planning.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface_msgs.msg import *
import rospy
import IPython



right_up_pose   = [0.284, -0.058, 0.631, -1.203, 0.547, 0.719, -0.753]
right_down_pose = [0.052, 0.173, 0.256, -1.299, 0.516, 0.742, -0.753]

right_one = [-0.588, 0.552, 0.291, -0.397, 1.835, 0.442, 0.17]
right_two = [-0.467, 1.115, 0.129, -0.595, 1.323, 0.677, -0.164]

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

    traj = vm.traj_from_path(plan_resp.path)
    vm.execute_trajectory(traj)

    vm.action_terminate_check_callback = None

    print("Checking torque limit")
    print hit_torque_limit
    if hit_torque_limit:
        # vm.plan_to_configuration(right_one, execute=True)
        print "Hit torque limit: Stopping"


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


    print("Planning right arm to configuration")
    vm.plan_to_configuration(right_one, execute=True)

    
    
    rospy.wait_for_service("plan_path")
    gpu_path = rospy.ServiceProxy("plan_path", gvpsrv.PlanPath)

    req = gvpsrv.PlanPathRequest()
    req.start = vu.list_to_jvq(right_one)
    req.goal = vu.list_to_jvq(right_two)

    
    try:
        plan_resp = gpu_path(req)

    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    success = execute_path(vm, plan_resp.path)


    

    if success:
        req = gvpsrv.PlanPathRequest()
        req.start = vu.list_to_jvq(right_two)
        req.goal = vu.list_to_jvq(right_one)

    
        try:
            plan_resp = gpu_path(req)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


        execute_path(vm, plan_resp.path)
    else:
        wiggle(vm)

    
    
    print("Finished")
    print("")
    
    
