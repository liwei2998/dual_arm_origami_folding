#!/usr/bin/env python
# coding: utf-8

# In[1]:


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import trajectory_msgs.msg 
import tf
import tf.transformations
from math import pi
from std_msgs.msg import String,Empty,UInt16
from moveit_commander.conversions import pose_to_list
import numpy as np
import helper
import math


# In[2]:


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
listener = tf.TransformListener()
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


# In[3]:


def addCollisionObjects():
    global box_pose1
    global box_name1
    box_pose1 = geometry_msgs.msg.PoseStamped()
    box_pose1.header.frame_id = "world"
    box_pose1.pose.orientation.w = 1.0
    box_pose1.pose.position.x = -0.105
    box_pose1.pose.position.y = -0.1485
    box_pose1.pose.position.z = 0.71
    box_name1 = 'box1'
    scene.add_box(box_name1, box_pose1, (0.01, 0.01, 0.01))
    global box_pose2
    global box_name2
    box_pose2 = geometry_msgs.msg.PoseStamped()
    box_pose2.header.frame_id = "world"
    box_pose2.pose.orientation.w = 1.0
    box_pose2.pose.position.x = 0.105
    box_pose2.pose.position.y = -0.1485
    box_pose2.pose.position.z = 0.71
    box_name2 = 'box2'
    scene.add_box(box_name2, box_pose2, (0.01, 0.01, 0.01))
    global box_pose3
    global box_name3
    box_pose3 = geometry_msgs.msg.PoseStamped()
    box_pose3.header.frame_id = "world"
    box_pose3.pose.orientation.w = 1.0
    box_pose3.pose.position.x = 0.105
    box_pose3.pose.position.y = 0.1485
    box_pose3.pose.position.z = 0.71
    box_name3 = 'box3'
    scene.add_box(box_name3, box_pose3, (0.01, 0.01, 0.01))
    global box_pose4
    global box_name4
    box_pose4 = geometry_msgs.msg.PoseStamped()
    box_pose4.header.frame_id = "world"
    box_pose4.pose.orientation.w = 1.0
    box_pose4.pose.position.x = -0.105
    box_pose4.pose.position.y = 0.1485
    box_pose4.pose.position.z = 0.71
    box_name4 = 'box4'
    scene.add_box(box_name4, box_pose4, (0.01, 0.01, 0.01))
    global paper_pose
    global paper_name
    paper_pose = geometry_msgs.msg.PoseStamped()
    paper_pose.header.frame_id = "world"
    paper_pose.pose.orientation.w = 1.0
    paper_pose.pose.position.x = 0
    paper_pose.pose.position.y = 0
    paper_pose.pose.position.z = 0.706
    paper_name = 'paper'
    scene.add_box(paper_name, paper_pose, (0.21, 0.297, 0.002))   
    global crease_pose1
    global crease_name1
    crease_pose1 = geometry_msgs.msg.PoseStamped()
    crease_pose1.header.frame_id = "world"
    crease_pose1.pose.orientation.x = 0
    crease_pose1.pose.orientation.y = 0
    crease_pose1.pose.orientation.z = -0.38268
    crease_pose1.pose.orientation.w = 0.9238777
    crease_pose1.pose.position.x = -0.03075
    crease_pose1.pose.position.y = -0.07425
    crease_pose1.pose.position.z = 0.7065
    crease_name1 = 'crease1'
    global crease_pose2
    global crease_name2
    crease_pose2 = geometry_msgs.msg.PoseStamped()
    crease_pose2.header.frame_id = "world"
    crease_pose2.pose.orientation.x = 0
    crease_pose2.pose.orientation.y = 0
    crease_pose2.pose.orientation.z = 0.38268
    crease_pose2.pose.orientation.w = 0.9238777
    crease_pose2.pose.position.x = -0.03075
    crease_pose2.pose.position.y = 0.07425
    crease_pose2.pose.position.z = 0.7065
    crease_name2 = 'crease2'    
    global crease_pose3
    global crease_name3
    crease_pose3 = geometry_msgs.msg.PoseStamped()
    crease_pose3.header.frame_id = "world"
    crease_pose3.pose.orientation.w = 1
    crease_pose3.pose.position.x = 0
    crease_pose3.pose.position.y = -0.07425
    crease_pose3.pose.position.z = 0.7065
    crease_name3 = 'crease3' 
    global wall_pose1
    global wall_name1
    wall_pose1 = geometry_msgs.msg.PoseStamped()
    wall_pose1.header.frame_id = "world"
    wall_pose1.pose.orientation.w = 1
    wall_pose1.pose.position.x = 0
    wall_pose1.pose.position.y = 0.75
    wall_pose1.pose.position.z = 1
    wall_name1 = "wall1"
    global wall_pose2
    global wall_name2
    wall_pose2 = geometry_msgs.msg.PoseStamped()
    wall_pose2.header.frame_id = "world"
    wall_pose2.pose.orientation.w = 1
    wall_pose2.pose.position.x = 0
    wall_pose2.pose.position.y = -0.75
    wall_pose2.pose.position.z = 1
    wall_name2 = "wall2"  
#     global wall_pose3
#     global wall_name3
#     wall_pose3 = geometry_msgs.msg.PoseStamped()
#     wall_pose3.header.frame_id = "world"
#     wall_pose3.pose.orientation.w = 1
#     wall_pose3.pose.position.x = 0
#     wall_pose3.pose.position.y = 0
#     wall_pose3.pose.position.z = 1.85
#     wall_name3 = "wall3"
#     scene.add_box(wall_name3,wall_pose3,(1.2,0.04,1.3))


# In[4]:


def robkong_go_to_home(times):
    if times == 1:
        joint_values = [-1.64, -0.73, 1.2, -1.94, -1.64, 0.73]
    elif times == 2:
        joint_values = [-2.03, -1.1, 1.93, -2.42, -1.5, 1.23]
    group2.set_joint_value_target(joint_values)
    group2.go(joint_values, wait=True)
    group2.stop()
    group2.clear_pose_targets()

def robhong_go_to_home(times):
    if times == 1:
        joint_values = [-1.41, -1.49, 2.2, -2.28, -1.57, 0.16]
    elif times == 2:
        joint_values = [-2.04,-1.28,2.33,-2.69,-1.56,0.84]
    group1.set_joint_value_target(joint_values)
    group1.go(joint_values, wait=True)
    group1.stop()
    group1.clear_pose_targets()


# In[5]:


def move_waypoints(dx,dy,dz,vel,robot_arm):
    '''
    This function is used for moving position of tool0 link of robot_arm.
    Input: difference between target position and initial position
    Output: execute a straight line trajectory between target and initial position
    '''
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0        
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    wpose = copy.deepcopy(group.get_current_pose().pose)
    wpose.position.x += dx
    wpose.position.y += dy
    wpose.position.z += dz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    if fraction == 1:
        # if plan successful, execute and return 1
        new_traj = scale_trajectory_speed(plan,vel)
        group.execute(new_traj)    
        return 1
    else:
        # else return 0
        return 0
    
def rot_waypoints(initial_quat,target_quat,vel,robot_arm):
    '''
    This function is used for rotating orientation of tool0 link of robot_arm.
    Input: initial quat and target quat
    Output: pure rotation of tool0 link of robot arm
    '''
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0 
    pose_target = group.get_current_pose().pose
    waypoints = []
    for t in range(0, 4):
        quat_waypoints = tf.transformations.quaternion_slerp(initial_quat,target_quat,t/3)
        pose_target.orientation.x = quat_waypoints[0]
        pose_target.orientation.y = quat_waypoints[1]
        pose_target.orientation.z = quat_waypoints[2]
        pose_target.orientation.w = quat_waypoints[3]
        waypoints.append(copy.deepcopy(pose_target))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)
    if fraction == 1:
        new_traj = scale_trajectory_speed(plan,vel)
        group.execute(new_traj)
        return 1
    else:
        return 0

# def move_target(target_pose, vel, robot_arm):
#     if robot_arm == "robhong":
#         group = group1
#     elif robot_arm == "robkong":
#         group = group2
#     else:
#         print "robot_arm input error, please input valid robot_arm name"
#         return 0 
#     pose_goal = geometry_msgs.msg.Pose()
#     pose_goal.position.x = target_pose[0]
#     pose_goal.position.y = target_pose[1]
#     pose_goal.position.z = target_pose[2]
#     pose_goal.orientation.x = target_pose[3]
#     pose_goal.orientation.y = target_pose[4]
#     pose_goal.orientation.z = target_pose[5]
#     pose_goal.orientation.w = target_pose[6]
#     plan0 = group.set_pose_target(pose_goal)
#     plan = group.plan(plan0)

#     if len(plan.joint_trajectory.points) > 0 :
#         new_traj = scale_trajectory_speed(plan,vel)
#         group.execute(plan)
#         group.clear_pose_targets()
#         return 1
#     except:
#         group.clear_pose_targets()
#         return 0

def scale_trajectory_speed(traj, scale):
    new_traj = moveit_msgs.msg.RobotTrajectory()
    new_traj.joint_trajectory = traj.joint_trajectory
    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)
    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point

    new_traj.joint_trajectory.points = points
    return new_traj


# In[42]:


def new_move_waypoints(target_position,source_frame,vel,robot_arm):
    '''
    This function is used for moving position of source_frame link of robot_arm. 2 steps: calculation and move.
    target_pos: position in world frame. position of source_frame.
    '''
    if robot_arm == "robhong":
        group = group1
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0 
    
    #step1: transformation, calculate the move distance between target_position and current source_frame position
    listener.waitForTransform("world",source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans1,rot1) = listener.lookupTransform("world",source_frame, rospy.Time(0))
    listener.waitForTransform( "world",target_frame,rospy.Time(), rospy.Duration(4.0))
    (trans2,rot2) = listener.lookupTransform("world",target_frame,rospy.Time(0))
    trans_target_source = np.subtract(trans1,trans2).tolist()
    current_pos = group.get_current_pose()
    delta = [target_position[0]-current_pos.pose.position.x-trans_target_source[0],
            target_position[1]-current_pos.pose.position.y-trans_target_source[1],
            target_position[2]-current_pos.pose.position.z-trans_target_source[2]]
    
    #step2: move waypoints in a straight line to the target_position
    waypoints = []
    waypoints.append(current_pos.pose)
    wpose = copy.deepcopy(current_pos.pose)
    wpose.position.x += delta[0]
    wpose.position.y += delta[1]
    wpose.position.z += delta[2]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    if fraction == 1:
        # if plan successful, execute and return 1
        new_traj = scale_trajectory_speed(plan,vel)
        group.execute(new_traj) 
        group.clear_pose_targets()
        print "Move waypoints to the target position succeed!"
        return 1
    else:
        print "Move waypoints to the target position failed..."
        return 0
    
def new_rot_waypoints(target_quat,source_frame,vel,robot_arm):
    '''
    This function is used for rotating orientation of source_frame link of robot_arm. 2 steps: calculation and move.
    target_quat: quat in world frame. quat of source_frame.
    '''
    if robot_arm == "robhong":
        group = group1
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0 
    #step1: calculate the goal_quat of ${robot_arm}_tool0
    listener.waitForTransform(source_frame,target_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
    goal_quat = tf.transformations.quaternion_multiply(target_quat,rot) #goal_quat
    current_pos =group.get_current_pose().pose
    initial_quat = [current_pos.orientation.x,current_pos.orientation.y,
                    current_pos.orientation.z,current_pos.orientation.w] #initial_quat

    #step2: rotate waypoints to the target orientation
    waypoints = []
    for t in range(0, 4):
        quat_waypoints = tf.transformations.quaternion_slerp(initial_quat,goal_quat,t/3)
        current_pos.orientation.x = quat_waypoints[0]
        current_pos.orientation.y = quat_waypoints[1]
        current_pos.orientation.z = quat_waypoints[2]
        current_pos.orientation.w = quat_waypoints[3]
        waypoints.append(copy.deepcopy(current_pos))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)
    if fraction == 1:
        new_traj = scale_trajectory_speed(plan,vel)
        group.execute(new_traj)
        group.clear_pose_targets()
        print "Rotate waypoints to the target orientation succeed!"
        return 1
    else:
        print "Rotate waypoints to the target orientation failed..."
        return 0

def new_move_target(target_pose,source_frame, vel, robot_arm):
    if robot_arm == "robhong":
        group = group1
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0 
    #step1: transformation, calculate the move distance between target_pos and source_frame
    listener.waitForTransform("world",source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans1,rot1) = listener.lookupTransform("world",source_frame, rospy.Time(0))
    listener.waitForTransform( "world",target_frame,rospy.Time(), rospy.Duration(4.0))
    (trans2,rot2) = listener.lookupTransform("world",target_frame,rospy.Time(0))
    trans_target_source = np.subtract(trans1,trans2).tolist()
    current_pos = group.get_current_pose()
    delta = [target_pose[0]-trans_target_source[0],
            target_pose[1]-trans_target_source[1],
            target_pose[2]-trans_target_source[2]]
    #step2: transformation: calculate the goal_quat of ${robot_arm}_tool0
    listener.waitForTransform(source_frame,target_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
    target_quat = [target_pose[3],target_pose[4],target_pose[5],target_pose[6]]
    goal_quat = tf.transformations.quaternion_multiply(target_quat,rot) #goal_quat
    #step3: move to the target pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = goal_quat[0]
    pose_goal.orientation.y = goal_quat[1]
    pose_goal.orientation.z = goal_quat[2]
    pose_goal.orientation.w = goal_quat[3]
    pose_goal.position.x = delta[0]
    pose_goal.position.y = delta[1]
    pose_goal.position.z = delta[2]
    plan0 = group.set_pose_target(pose_goal)
    plan = group.plan(plan0)
    if len(plan.joint_trajectory.points) > 0 :
        new_traj = scale_trajectory_speed(plan,vel)
        group.execute(plan)
        print "Move targets to the target pose succeed!"
        group.clear_pose_targets()
        return 1
    else:
        print "Move targets to the target pose failed..."
        return 0
    
def scale_trajectory_speed(traj, scale):
    new_traj = moveit_msgs.msg.RobotTrajectory()
    new_traj.joint_trajectory = traj.joint_trajectory
    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)
    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point

    new_traj.joint_trajectory.points = points
    return new_traj


# In[43]:


def move_autonomously(target_pose,source_frame,vel,robot_arm):
    '''
    This function is used for autonomously moving.
    It involves 3 move functions: new_move_waypoints(), new_rot_waypoints(), move_target().
    The simple logic of this function is to try to implement new_rot_waypoints() and new_move_waypoints() first,
    if the above two move functions failed, try to implement move_target() instead.
    '''
    if robot_arm == "robhong":
        group = group1
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0        
    target_position = [target_pose[0],target_pose[1],target_pose[2]]
    target_quat = [target_pose[3],target_pose[4],target_pose[5],target_pose[6]]

    is_rot_waypoints = new_rot_waypoints(target_quat,source_frame,vel,robot_arm)

    if is_rot_waypoints == 1:
        rospy.sleep(1)
        is_move_waypoints = new_move_waypoints(target_position,source_frame,vel,robot_arm)

        if is_move_waypoints == 1:
            rospy.sleep(1)
            return 1

        elif is_move_waypoints == 0:
            rospy.sleep(1)
            is_move_target = new_move_target(target_pose,source_frame, vel,robot_arm)

            if is_move_target == 1:
                rospy.sleep(1)
                return 1

            elif is_move_target == 0:
                print "all moving methods failed..."
                return 0
    
    elif is_rot_waypoints == 0: 
        rospy.sleep(1)
        is_move_waypoints = new_move_waypoints(target_position,source_frame,vel,robot_arm)
        
        if is_move_waypoints == 1:    
            rospy.sleep(1)
            is_rot_waypoints = new_rot_waypoints(target_quat,source_frame,vel,robot_arm)

            if is_rot_waypoints == 1:
                rospy.sleep(1)
                return 1
            
            elif is_rot_waypoints == 0:
                rospy.sleep(1)
                is_move_target = new_move_target(target_pose,source_frame, vel,robot_arm)

                if is_move_target == 1:
                    rospy.sleep(1)
                    return 1

                elif is_move_target == 0:
                    print "all moving methods failed..." 
                    return 0   
                
        elif is_move_waypoints == 0:
            rospy.sleep(1)
            is_move_target = new_move_target(target_pose,source_frame, vel,robot_arm)

            if is_move_target == 1:
                rospy.sleep(1)
                return 1

            elif is_move_target == 0:
                print "all moving methods failed..." 
                return 0


# In[7]:


def tilt(point, axis, angle, velocity, robot_arm):
    '''
    Parameters:
        point (list): 3-D coordinate of point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tilting
        velocity (double): robot velocity between 0 and 1
    '''
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0  
    # Normalize axis vector
    axis = axis / np.linalg.norm(axis)
    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]

    # Tilt center point. Closest point from tcp to axis line
    center = np.add(point, np.dot(np.subtract(pos_initial, point), axis)*axis)

    # Closest distance from tcp to axis line
    radius = np.linalg.norm(np.subtract(center, pos_initial))

    # Pair of orthogonal vectors in tilt plane
    v1 =  -np.subtract(np.add(center, np.dot(np.subtract(pos_initial, center), axis)*axis), pos_initial)
    v1 = v1/np.linalg.norm(v1)
    v2 = np.cross(axis, v1)

    # Interpolate orientation poses via quaternion slerp
    q = helper.axis_angle2quaternion(axis, angle)
    ori_target = tf.transformations.quaternion_multiply(q, ori_initial)
    ori_waypoints = helper.slerp(ori_initial, ori_target, np.arange(1.0/angle , 1.0+1.0/angle, 1.0/angle))

    waypoints = []
    for t in range(1, angle + 1):
        circle = np.add(center, radius*(math.cos(math.radians(t)))*v1 + radius*(math.sin(math.radians(t)))*v2)
        pose_target.position.x = circle[0]
        pose_target.position.y = circle[1]
        pose_target.position.z = circle[2]
        pose_target.orientation.x = ori_waypoints[t-1][0]
        pose_target.orientation.y = ori_waypoints[t-1][1]
        pose_target.orientation.z = ori_waypoints[t-1][2]
        pose_target.orientation.w = ori_waypoints[t-1][3]
        waypoints.append(copy.deepcopy(pose_target))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) # waypoints, resolution=1cm, jump_threshold)
    plan = scale_trajectory_speed(plan,velocity)
    group.execute(plan)


# In[40]:


def fix_paper(paper_edge_point1,paper_edge_point2,robot_arm):
    '''
    This function is  used for fixing paper.
    steps: calculate taget pose; move to the target pose.
    orientation: y axis of source_frame will be aligned with [point2 -> point1], 
                 z axis of source_frame is [0,0,-1] in world frame.
    position: origin of source_frame will be located at the mid of point1 and point2
    '''
    if robot_arm == "robhong":
        group = group1
        source_frame = "soft_gripper_hong"
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        source_frame = "soft_gripper_kong"
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    #step1: calculate the target pose for the source_frame
    target_pose = []
    # a): target position
    paper_edge_mid = np.subtract(paper_edge_point1,paper_edge_point2)*0.5 + np.array(paper_edge_point2)
    paper_edge_mid = paper_edge_mid.tolist()
    target_pose = target_pose + paper_edge_mid
    target_pose[2] = target_pose[2] + 0.07 #this is to ensure no collision between the source_frame and the table

    # b): target quat  
    paper_axis = np.subtract(paper_edge_point1,paper_edge_point2).tolist()
    paper_axis = paper_axis / np.linalg.norm(paper_axis)
    paper_axis = paper_axis.tolist()
    x_axis = [0,0,-1]
    z_axis = np.cross(x_axis, paper_axis).tolist()
    y_axis = paper_axis
    x_axis.append(0)
    y_axis.append(0)
    z_axis.append(0)
    target_rot_mat = np.array([x_axis,
                     y_axis,
                     z_axis,
                     [0,0,0,1]]).transpose()
    target_rot_mat = target_rot_mat.tolist()
    target_quat = tf.transformations.quaternion_from_matrix(target_rot_mat).tolist()
    
    target_pose = target_pose + target_quat
    
    #step2: move to the target pose
    move_autonomously(target_pose,source_frame,0.4,robot_arm)


# In[44]:


def flex_flip(paper_edge_point,paper_edge_axis,robot_arm):
    '''
    This function is used for flex-flip.
    steps: calculate target pose; move to the target pose; additional rotate for closing gripper.
    orientation: angle between y axis of source_frame and paper_edge_axis is pi/4 (+:counterclockwise),
                 x axis of source_frame is [0,0,-1] in world frame.
    position: origin of source_frame will be located at paper_edge_point
    '''
    if robot_arm == "robhong":
        group = group1
        source_frame = "soft_gripper_hong"
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        source_frame = "soft_gripper_kong"
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    #step1: calculate the target pose for the source_frame
    # a): target position
    target_pose = []
    target_pose = target_pose + paper_edge_point
    target_pose[2] = target_pose[2] + 0.05 #this is to ensure no collision between the source_frame and the table
    # b): target quat    
    paper_edge_axis = paper_edge_axis / np.linalg.norm(paper_edge_axis)
    paper_edge_axis = paper_edge_axis.tolist()
    x_axis = [0,0,-1]
    z_axis = paper_edge_axis
    y_axis = np.cross(z_axis,x_axis).tolist()
    z_axis = paper_edge_axis
    x_axis.append(0)
    y_axis.append(0)
    z_axis.append(0)
    rot_mat_temp = np.array([x_axis,
                     y_axis,
                     z_axis,
                     [0,0,0,1]]).transpose()
    rot_mat_temp = rot_mat_temp.tolist()
    rot_mat = tf.transformations.rotation_matrix(pi/4,(1,0,0))
    target_rot_mat = np.dot(rot_mat_temp,rot_mat).tolist()
    target_quat = tf.transformations.quaternion_from_matrix(target_rot_mat).tolist()
    
    target_pose = target_pose + target_quat
    
    #step2: move autonomosly to the target pose
    move_autonomously(target_pose,source_frame,0.4,robot_arm)
    
    #step3: rotate around z axis of source_frame, and around the origin of source_frame
    listener.waitForTransform("world",source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("world",source_frame, rospy.Time(0))
    center = trans[0:3] #center locates at origin of source_frame
    phi = 15
    rot = tf.transformations.quaternion_matrix(rot).tolist()
    axis = np.dot(rot,[0,0,-1,1]).tolist()[0:3] #z_axis of soft_gripper_$(robot_arm) frame
    tilt(center,axis,phi,0.5,robot_arm)


# In[45]:


def fold(crease_point,crease_axis,fold_angle,move_dist,robot_arm,vel=0.4):
    #step1: rotate around the crease line
    tilt(crease_point,crease_axis,fold_angle,0.5,robot_arm)
    #step2: move to the target position
    x_axis = [0,0,-1]
    crease_axis = crease_axis / np.linalg.norm(crease_axis)
    direction = np.cross(x_axis,crease_axis)
    direction = direction*move_dist
    move_waypoints(direction[0],direction[1],direction[2],0.4,robot_arm)


# In[11]:


def scooping(scooping_point,scooping_axis,scooping_angle,robot_arm,scooping_dis=0.04):
    '''
    This function is used for scooping paper, has 3 steps:rotate to scooping orientation, move to scooping position,
        and move towards scooping direction at the distance of scooping_dis to scoop paper.
    Orientation: y axis of source_frame will be aligned with scooping_axis, scooping_angle (rad) is the angle between 
        ground and rigid finger of robot_arm.
    Position: origin of source_frame will be located at scooping point.
    Direction: according to right-hand rule, cross product of ([0,0,-1],scooping_axis) in world
    ''' 
    if robot_arm == "robhong":
        group = group1
        source_frame = "rigid_tip_link1"
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        source_frame = "rigid_tip_link2"
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    # step1: rotate to scooping orientation
    scooping_axis = scooping_axis / np.linalg.norm(scooping_axis)
    scooping_axis = scooping_axis.tolist()    
    x_axis = [0,0,-1]
    z_axis = np.cross(x_axis, scooping_axis).tolist()
    y_axis = scooping_axis
    x_axis.append(0)
    y_axis.append(0)
    z_axis.append(0)

    rot_mat_temp = np.array([x_axis,
                     y_axis,
                     z_axis,
                     [0,0,0,1]]).transpose()
    rot_mat_temp = rot_mat_temp.tolist()
    #tilt_angle: rad, i.e. pi/4
    rot_mat = tf.transformations.rotation_matrix(scooping_angle,(0,1,0))
    target_rot_mat = np.dot(rot_mat_temp,rot_mat).tolist()
    
    listener.waitForTransform(source_frame,target_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
    rot = tf.transformations.quaternion_matrix(rot)
    target_rot_mat = np.dot(target_rot_mat,rot).tolist()
    target_quat = tf.transformations.quaternion_from_matrix(target_rot_mat)
    s =group.get_current_pose().pose.orientation
    initial_quat = [s.x,s.y,s.z,s.w]
    rot_waypoints(initial_quat,target_quat,0.4,robot_arm)
    rospy.sleep(1)
    # step2: move to scooping position
    listener.waitForTransform("world",source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans1,rot1) = listener.lookupTransform("world",source_frame, rospy.Time(0))
    listener.waitForTransform( "world",target_frame,rospy.Time(), rospy.Duration(4.0))
    (trans2,rot2) = listener.lookupTransform("world",target_frame,rospy.Time(0))
    trans_world_tip = np.subtract(trans1,trans2).tolist()    
    descend_height = trans1[2]-0.72 #0.705 is the height of movable table
    current_pos = group.get_current_pose()
    delta = [scooping_point[0]-current_pos.pose.position.x-trans_world_tip[0],
            scooping_point[1]-current_pos.pose.position.y-trans_world_tip[1],
            -descend_height]
    move_waypoints(delta[0],delta[1],delta[2],0.4,robot_arm)
    rospy.sleep(1)
    # step3: move towards scooping direction
    direction = np.array(z_axis[0:3])
    direction = direction*scooping_dis
    move_waypoints(direction[0],direction[1],direction[2],0.4,robot_arm)


# In[12]:


def make_crease(paper_edge_point1,paper_edge_point2,robot_arm,n=5):
    '''
    This function is used for making a crease, has 2 steps:rotate to crease-making orientation, 
        move to crease-making position.
    Orientation: y axis of source_frame will be aligned with [point2 -> point1]
    Position: origin of source_frame will be located at point2
    '''
    if robot_arm == "robhong":
        group = group1
        source_frame = "rigid_tip_link1"
        target_frame = "hong_tool0"
    elif robot_arm == "robkong":
        group = group2
        source_frame = "rigid_tip_link2"
        target_frame = "kong_tool0"
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    #step1: rotate to crease-making orientation
    paper_axis = np.subtract(paper_edge_point1,paper_edge_point2).tolist()
    paper_axis = paper_axis / np.linalg.norm(paper_axis)
    paper_axis = paper_axis.tolist()
    x_axis = [0,0,-1]
    z_axis = np.cross(x_axis, paper_axis).tolist()
    y_axis = paper_axis
    x_axis.append(0)
    y_axis.append(0)
    z_axis.append(0)

    rot_mat_temp = np.array([x_axis,
                     y_axis,
                     z_axis,
                     [0,0,0,1]]).transpose()
    rot_mat_temp = rot_mat_temp.tolist()
    rot_mat = tf.transformations.rotation_matrix(pi/4,(0,1,0))
    target_rot_mat = np.dot(rot_mat_temp,rot_mat).tolist()
    
    listener.waitForTransform(source_frame,target_frame, rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
    rot = tf.transformations.quaternion_matrix(rot)
    target_rot_mat = np.dot(target_rot_mat,rot).tolist()
    target_quat = tf.transformations.quaternion_from_matrix(target_rot_mat)
    s =group.get_current_pose().pose.orientation
    initial_quat = [s.x,s.y,s.z,s.w]
    rot_waypoints(initial_quat,target_quat,0.4,robot_arm)
    rospy.sleep(1)
    #step2: move to crease-making position
    interval = np.subtract(paper_edge_point2,paper_edge_point1) / n
    listener.waitForTransform("world",source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans1,rot1) = listener.lookupTransform("world",source_frame, rospy.Time(0))
    listener.waitForTransform( "world",target_frame,rospy.Time(), rospy.Duration(4.0))
    (trans2,rot2) = listener.lookupTransform("world",target_frame,rospy.Time(0))
    trans_world_tip = np.subtract(trans1,trans2).tolist()    
    descend_height = trans1[2]-0.72 #0.705 is the height of movable table
    move_waypoints(0,0,-descend_height,0.4,robot_arm)
    for i in range(n+1):
        point = np.array(paper_edge_point1) + i*interval
        point = point.tolist()
        current_pos = group.get_current_pose()
        delta = [point[0]-current_pos.pose.position.x-trans_world_tip[0],
                point[1]-current_pos.pose.position.y-trans_world_tip[1],
                0]
        move_waypoints(delta[0],delta[1],delta[2],0.4,robot_arm) 
        rospy.sleep(1)
        direction = np.array(z_axis[0:3])
        direction = direction*0.02
        move_waypoints(direction[0],direction[1],direction[2],0.4,robot_arm)
        rospy.sleep(1)
        move_waypoints(-direction[0],-direction[1],-direction[2],0.4,robot_arm)


# In[18]:


# scene.add_box(crease_name1, crease_pose1, (0.21, 0.01, 0.005))
# scene.add_box(crease_name2, crease_pose2, (0.21, 0.01, 0.005))
# scene.remove_world_object(crease_name1)
# scene.remove_world_object(crease_name2)
# scene.remove_world_object(crease_name3)
scene.remove_world_object(wall_name1)
scene.remove_world_object(wall_name2)
# scene.add_box(crease_name3, crease_pose3, (0.21, 0.01, 0.005))
# robkong_go_to_home(2)
# robhong_go_to_home(2)
# fix_paper(point1,point2,"robkong")
# flex_flip(point4,edge4,"robhong")
# fold(point3,crease_axis2,30,0.1, "robhong")
# move_waypoints(0.2,0.1,0,0.4,"robkong")
# scene.add_box(crease_name2, crease_pose2, (0.21, 0.01, 0.005))
# make_crease(crease_point3,crease_point1,"robkong")
# scene.add_box(crease_name2, crease_pose2, (0.21, 0.01, 0.005))
# fix_paper(point1,point2,"robhong")
# flex_flip(point4,edge4,"robkong")
# fold(point3,crease_axis2,30,0.05, "robkong")


# In[9]:


# We can get the name of the reference frame for this robot:
planning_frame = group1.get_planning_frame()
print "============ Reference frame1: %s" % planning_frame
eef_link = group1.get_end_effector_link()
print "============ End effector1: %s" % eef_link
planning_frame = group2.get_planning_frame()
print "============ Reference frame2: %s" % planning_frame
eef_link = group2.get_end_effector_link()
print "============ End effector2: %s" % eef_link
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()


# In[36]:


#_____INITIALIZATION_____#
addCollisionObjects()
rospy.sleep(1)
point1 = [box_pose1.pose.position.x,box_pose1.pose.position.y,box_pose1.pose.position.z]
point2 = [box_pose2.pose.position.x,box_pose2.pose.position.y,box_pose2.pose.position.z]
point3 = [box_pose3.pose.position.x,box_pose3.pose.position.y,box_pose3.pose.position.z]
point4 = [box_pose4.pose.position.x,box_pose4.pose.position.y,box_pose4.pose.position.z]
edge1 = np.subtract(point1,point2).tolist()
edge2 = np.subtract(point2,point3).tolist()
edge3 = np.subtract(point3,point4).tolist()
edge4 = np.subtract(point4,point1).tolist()
crease_axis1 = [-0.7071068,0.7071068,0]
crease_axis2 = [0.7071068,0.7071068,0]
crease_axis3 = [-1,0,0]
crease_point1 = [-0.105,0,0.76]
crease_point2 = [0.0435,-0.1485,0.76]
crease_point3 = [0.0435,0.1485,0.76]
crease_point4 = [0.105,-0.07425,0.76]
crease_point5 = [-0.015375,-0.07425,0.76]


# In[48]:


#_____INITIALIZATION______#
robkong_go_to_home(1)
robhong_go_to_home(1)
scene.add_box(wall_name1,wall_pose1,(1.2,0.04,2))
scene.add_box(wall_name2,wall_pose2,(1.2,0.04,2)) 


# In[49]:


fix_paper(point3,point4,"robhong")


# In[50]:


flex_flip(point1,edge1,"robkong")


# In[136]:


fold(point2, crease_axis1,30,0.1, "robkong")


# In[137]:


scene.add_box(crease_name1, crease_pose1, (0.21, 0.01, 0.005))
make_crease(crease_point1,crease_point2,"robhong",2)


# In[39]:


fix_paper(point3,point4,"robkong")


# In[27]:


scooping(crease_point2,crease_axis3,pi/4,"robhong")


# In[29]:


fold(crease_point4, crease_axis3,60,0.1, "robhong",0.08)


# In[20]:


scene.add_box(crease_name3, crease_pose3, (0.21, 0.01, 0.005))
make_crease(crease_point5,crease_point4,"robkong",2)


# In[28]:


# robkong_go_to_home(2)
# fix_paper(point3,point4,"robkong")
# tar = [-0.075,-0.174,0.833,-0.670,0.670,0.227,0.227]
# move_autonomously(tar,"rigid_tip_link2",0.4,"robkong")


# In[129]:


#solution 1: generate a part of trajectory by us to avoid collsion
move_waypoints(0.35,0,0,0.4,"robkong") #first move in the x direction
move_waypoints(0,-0.35,0,0.4,"robkong") #second move in the y direction
rospy.sleep(1)
make_crease(crease_point5,crease_point4,"robkong",2)


# In[126]:


tar = [-0.075,-0.174,0.833,-0.670,0.670,0.227,0.227]
move_autonomously(tar,"rigid_tip_link2",0.4,"robkong")


# In[32]:


#solution 2: generate the whole trajectory in moveit to avoid collision
group2.clear_pose_targets()
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = -0.66976218
pose_goal.orientation.y = 0.6697671
pose_goal.orientation.z = 0.22675032
pose_goal.orientation.w = 0.22674866
pose_goal.position.x = -0.0751493723373
pose_goal.position.y = -0.173646802403
pose_goal.position.z = 0.833043830035
# pose_goal.position.x = 0
# pose_goal.position.y = 0
# pose_goal.position.z = 0.8
myplan = group2.set_pose_target(pose_goal)
success = group2.plan(myplan)
new_traj = scale_trajectory_speed(success,0.4)
group2.execute(new_traj)

# group2.clear_pose_targets()
# print len(success.joint_trajectory.points) >0


# In[73]:


waypoints = []
waypoints.append(group2.get_current_pose().pose)
wpose = copy.deepcopy(group2.get_current_pose().pose)
wpose.position.x += 0
wpose.position.y -= 0.2
wpose.position.z += 0
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = group2.compute_cartesian_path(waypoints, 0.01, 0.0, True)
# success = group2.plan(plan)
print fraction
group2.execute(plan)


# In[ ]:




