#! /usr/bin/env python

# from openravepy import *
from arm_robots.victor import Victor
# import arm_or_robots.motion_victor
# import victor_hardware_interface.msg as vhimsg
import gpu_voxel_planning_msgs.srv as gvpsrv
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface_msgs.msg import *
# from arc_utilities import ros_helpers as rh
from arc_utilities.listener import Listener
from arc_utilities import path_utils as pu
from gpu_voxel_planning_msgs.msg import CollisionInformation
from gpu_voxel_planning_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
from threading import Lock
import copy
from colorama import Fore

voice = None
check_collision_lock = Lock()
start_new_path_lock = Lock()

g_in_collision = False
g_links_in_contact = []
last_collision_info = CollisionInformation()
ros_path = []
path_in_progress = False

ext_torque_limits = [10, 5, 4, 5, 4, 3, .9]

right_arm_listener = None


def check_collision(motion_status_msg):
    global g_in_collision
    global g_links_in_contact
    with check_collision_lock:

        g_links_in_contact = []
        jt = vu.jvq_to_list(motion_status_msg.estimated_external_torque)

        col_indicies = [idx for idx in range(len(jt))
                        if abs(jt[idx]) > ext_torque_limits[idx]
                        if idx > 0]

        if col_indicies:
            print(jt)
            print(col_indicies)

        g_in_collision = bool(col_indicies)

        if not g_in_collision:
            return False

        g_links_in_contact = ["victor_right_arm_link_" + str(idx + 1) for idx in range(col_indicies[-1], len(jt))]
        # print("check_collision links in contact")
        # print(g_links_in_contact)
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
    rospy.loginfo("Received new path attempt")

    resp = AttemptPathStartResponse()

    if path_in_progress:
        rospy.loginfo("But there is already a path in progress")

        resp.started = False
        return resp

    resp.started = True

    rospy.loginfo("Triggering new path start")
    with start_new_path_lock:
        path_in_progress = True
        ros_path = [point.positions for point in req.path.points]
    rospy.loginfo("Returning")
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

    rospy.loginfo("Attempting path")
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
    global victor
    global path_in_progress
    # global col_links

    rospy.loginfo("Executing path")
    col_links = []
    # global in_collision
    in_collision = False

    def stop(*args):
        global g_in_collision
        global g_links_in_contact
        nonlocal in_collision
        nonlocal col_links
        with check_collision_lock:
            if g_in_collision:
                # rospy.loginfo("collision during motion")
                in_collision = True
                col_links = copy.deepcopy(g_links_in_contact)

                print("stop links in contact")
                print(g_links_in_contact)
                print(col_links)
            return g_in_collision


    victor.set_execute(False)
    plan_result = victor.plan_to_joint_config("right_arm", path[-1])
    if not plan_result.success:
        raise RuntimeError("Error, unable to plan. This should not happen")
    victor.set_execute(True)

    # old_traj = plan_result.planning_result.plan.joint_trajectory
    #
    # traj = victor.joint_trajectory_from_path("right_arm", path)
    # traj = victor.joint_trajectory_from_path("right_arm", old_traj)
    result = victor.follow_arms_joint_trajectory(plan_result.planning_result.plan.joint_trajectory, stop_condition=stop)

    # result = victor.follow_arms_joint_trajectory(traj, stop_condition=stop)

    msg = CollisionInformation()
    msg.collided = in_collision

    if not in_collision:
        rospy.sleep(0.2)
        stop()

    if not in_collision:
        return msg

    # speak_collision_link()

    print("assignment links in contact")
    print(col_links)
    msg.collision_links = col_links

    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)

    _, cur_ind, _ = pu.closest_point(path, cur_pos)
    free_poses = path[0:cur_ind]
    next_poses = pu.densify(pu.travel_along(path, .15, cur_pos), .03)

    for pos in free_poses:
        jtp_msg = JointTrajectoryPoint()
        jtp_msg.positions = pos
        msg.free_path.points.append(jtp_msg)

    for pos in next_poses:
        jtp_msg = JointTrajectoryPoint()
        jtp_msg.positions = pos
        msg.collision_path.points.append(jtp_msg)

    backup_path = pu.travel_along(path, -0.1, cur_pos)

    rospy.loginfo("Collision detected. Backing up")
    victor.plan_to_joint_config('right_arm', backup_path[-1])
    # vm.execute_trajectory(vm.traj_from_path(backup_path))
    return msg


def go_to(config):
    cur_pos = vu.jvq_to_list(right_arm_listener.get().measured_joint_position)
    path = [cur_pos, config]
    return execute_path(path)


def speak_collision_link():
    while not rospy.is_shutdown():

        with check_collision_lock:
            play_msg = g_in_collision

            if play_msg:
                voice.publish("collision link " + g_links_in_contact[0][-1])
                rospy.sleep(1)


if __name__ == "__main__":

    rospy.init_node("execute_path_with_collision_detection")
    voice = rospy.Publisher("polly", String, queue_size=10)

    # vm = arm_or_robots.motion_victor.MotionEnabledVictor(viewer=False, world_frame="gpu_voxel_world")
    victor = Victor(display_goals=False)
    victor.connect()
    # vm.set_manipulator("left_arm")
    # vm.change_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=vu.Stiffness.MEDIUM)
    # vm.set_manipulator("right_arm")
    # vm.change_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=vu.Stiffness.MEDIUM)
    victor.set_control_mode(ControlMode.JOINT_IMPEDANCE, vel=0.1)

    sub = rospy.Subscriber("victor/right_arm/motion_status", MotionStatus, check_collision)
    s = rospy.Service("attempt_path_on_victor", AttemptPathStart, attempt_path_srv)
    s = rospy.Service("get_path_status", AttemptPathResult, path_status_srv)
    right_arm_listener = Listener("victor/right_arm/motion_status", MotionStatus)

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
