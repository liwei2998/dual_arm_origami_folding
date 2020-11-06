#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import trajectory_msgs.msg
import tf.transformations as tf
from math import pi
from std_msgs.msg import String,Empty,UInt16
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)
group_name1 = "hong_arm"
group1 = moveit_commander.MoveGroupCommander(group_name1)
group_name2 = "kong_arm"
group2 = moveit_commander.MoveGroupCommander(group_name2)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

def addCollisionObjects(box_is_attached = False, box_is_known = False, timeout = 4):
    box_pose2 = geometry_msgs.msg.PoseStamped()
    box_pose2.header.frame_id = "world"
    box_pose2.pose.orientation.w = 1.0
    box_pose2.pose.position.x = -0.05
    box_pose2.pose.position.y = -0.05
    box_pose2.pose.position.z = 0.706
    box_name = 'box'
    scene.add_box(box_name, box_pose2, (0.02, 0.02, 0.02))

def robkong_go_to_home():
    joint_values = [-1.64, -0.73, 1.2, -1.94, -1.64, 0.73]
    group2.set_joint_value_target(joint_values)
    group2.go(joint_values, wait=True)
    group2.stop()
    group2.clear_pose_targets()

def robhong_go_to_home():
    joint_values = [-1.41, -1.49, 2.2, -2.28, -1.57, 0.16]
    group1.set_joint_value_target(joint_values)
    group1.go(joint_values, wait=True)
    group1.stop()
    group1.clear_pose_targets()

def pick():
    grasps = moveit_msgs.msg.Grasp()

    #set grasp pose
    grasps.grasp_pose.header.frame_id = "kong_base_link"
    rotate_matrix = [[0,0.7071067,-0.7071067,0],
                    [0,-0.7071067,-0.7071067,0],
                    [-1,0,0,0],
                    [0,0,0,1]]
    ori = tf.quaternion_from_matrix(rotate_matrix)
    grasps.grasp_pose.pose.orientation = ori
    grasps.grasp_pose.pose.position.x = -0.05
    grasps.grasp_pose.pose.position.y = -0.05
    grasps.grasp_pose.pose.position.z = 0.846

    #set pre grasp approach
    grasps.pre_grasp_approach.direction.header.frame_id = "kong_base_link"
    grasps.pre_grasp_approach.direction.vector.z = -1
    grasps.pre_grasp_approach.min_distance = 0.03
    grasps.pre_grasp_approach.desired_distance = 0.05

    #set post grasp retreat
    grasps.post_grasp_retreat.direction.header.frame_id = "kong_wrist_link3"
    grasps.post_grasp_retreat.direction.vector.x = 1.0
    grasps.post_grasp_retreat.min_distance = 0.02
    grasps.post_grasp_retreat.desired_distance = 0.05

def move_waypoints(positions,robot_arm):
    waypoints = []
    waypoints.append(group1.get_current_pose().pose)
    wpose = copy.deepcopy(group1.get_current_pose().pose)
    wpose.position.x += positions.x
    wpose.position.y += positions.y
    wpose.position.z += positions.z
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group1.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
    if int(robot_arm) == 1:
        group1.go(plan)
    elif int(robot_arm) == 2:
        group2.go(plan)

if __name__=='__main__':
    addCollisionObjects()
    rospy.sleep(1)
    robkong_go_to_home()
    robhong_go_to_home()
    grasps = geometry_msgs.msg.PoseStamped()

    #set grasp pose
    grasps.header.frame_id = "world"

    rotate_matrix = [[0,-0.7071067,-0.7071067,0],
                    [0,-0.7071067,0.7071067,0],
                    [-1,0,0,0],
                    [0,0,0,1]]
    ori = tf.quaternion_from_matrix(rotate_matrix)
    print ori
    grasps.pose.orientation.x = ori[0]
    grasps.pose.orientation.y = ori[1]
    grasps.pose.orientation.z = ori[2]
    grasps.pose.orientation.w = ori[3]
    print grasps.pose.orientation
    grasps.pose.position.x = -0.05
    grasps.pose.position.y = -0.05
    grasps.pose.position.z = 0.846

    group2.set_pose_target(grasps)
    group2.go()

    #move_waypoints(grasps.pose.position)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    moveit_commander.os._exit(0)
