#! /usr/bin/env python

from openravepy import *
import victor_motion_planner.openrave_planner
# import victor_hardware_interface.msg as vhimsg
import victor_motion_planner.msg as vmmsg
import gpu_voxel_planning.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
import rospy
import IPython



right_up_pose   = [0.284, -0.058, 0.631, -1.203, 0.547, 0.719, -0.753]
right_down_pose = [0.052, 0.173, 0.256, -1.299, 0.516, 0.742, -0.753]

def get_jvq_path():
    path = vmmsg.Path()
    path.values.append(vu.list_to_jvq(right_down_pose))
    path.values.append(vu.list_to_jvq(right_up_pose))
    path.values.append(vu.list_to_jvq(right_down_pose))
    path.values.append(vu.list_to_jvq(right_up_pose))
    return path

if __name__ == "__main__":
    rospy.init_node("basic_motion")

    vm = victor_motion_planner.openrave_planner.Planner()
    vm.set_manipulator("right_arm")
    
    rospy.wait_for_service('plan_path')
    gpu_path = rospy.ServiceProxy('plan_path', gvpsrv.PlanPath)

    req = gvpsrv.PlanPathRequest()
    req.start = vu.list_to_jvq(right_up_pose)
    req.goal = vu.list_to_jvq(right_down_pose)

    plan_resp = None
    
    try:
        plan_resp = gpu_path(req)

    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    # path = get_jvq_path()
    traj = vm.traj_from_path(plan_resp.path)
    vm.execute_trajectory(traj)
    


    req = gvpsrv.PlanPathRequest()
    req.start = vu.list_to_jvq(right_down_pose)
    req.goal = vu.list_to_jvq(right_up_pose)

    
    try:
        plan_resp = gpu_path(req)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


    # path = get_jvq_path()
    traj = vm.traj_from_path(plan_resp.path)
    vm.execute_trajectory(traj)
    
