#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
# import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

# from whole_part import SceneObject

import yaml
import tf2_ros


class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        br = tf.TransformBroadcaster()

        #rospy.init_node('move_group_planner',
        #                anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.plan_scene = moveit_commander.PlanningScene()
        self.group_left = moveit_commander.MoveGroupCommander("panda_left")
        self.group_right = moveit_commander.MoveGroupCommander("panda_right")
        self.group_top = moveit_commander.MoveGroupCommander("panda_top")
        self.group_list = [self.group_left, self.group_right, self.group_top]
        self.group_123 = moveit_commander.MoveGroupCommander("panda_triple")
        self.hand_left = moveit_commander.MoveGroupCommander("hand_left")
        self.hand_right = moveit_commander.MoveGroupCommander("hand_right")
        self.hand_top = moveit_commander.MoveGroupCommander("hand_top")
        self.hand_list = [self.hand_left, self.hand_right, self.hand_top]

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_left.get_planning_frame()
        print("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_left.get_end_effector_link()
        print("============ End effector: %s" % self.eef_link)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_top.get_planning_frame()
        print("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_top.get_end_effector_link()
        print("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.robot.get_group_names())

        # self.scene.remove_attached_object(self.group_left.get_end_effector_link())
        # self.scene.remove_attached_object(self.group_top.get_end_effector_link())
        # self.scene.remove_world_object()
        rospy.sleep(1)
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base"
        self.box_pose.pose.orientation.w = 1.0
        self.box_pose.pose.position.x = 0.65
        self.box_pose.pose.position.y = 0.0
        self.box_pose.pose.position.z = 1.1
        self.scene.add_box("table", self.box_pose, size=(0.65, 1.0, 0.2))
        
        self.obstacle = geometry_msgs.msg.PoseStamped()
        self.obstacle.header.frame_id = "base"
        self.obstacle.pose.orientation.w = 1.0
        self.obstacle.pose.position.x = 0.65
        self.obstacle.pose.position.y = 0.1
        self.obstacle.pose.position.z = 1.3
        self.scene.add_box("obstacle", self.obstacle, size=(0.5, 0.08, 0.1))
        

        rospy.sleep(1)
        self.active_controllers = []
    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()

    def plan(self, goal, arm_name):
        if (arm_name == 'left'):
            self.group_left.set_max_velocity_scaling_factor = 0.6
            self.group_left.set_max_acceleration_scaling_factor = 0.4
            self.group_left.set_start_state_to_current_state()
            self.group_left.plan(goal)
            self.group_left.go()
        if (arm_name == 'top'):
            self.group_top.set_max_velocity_scaling_factor = 0.6
            self.group_top.set_max_acceleration_scaling_factor = 0.4
            self.group_top.set_start_state_to_current_state()
            trajectory = self.group_top.plan(goal)
        if (arm_name == 'right'):
            self.group_right.set_max_velocity_scaling_factor = 0.6
            self.group_right.set_max_acceleration_scaling_factor = 0.4
            self.group_right.set_start_state_to_current_state()
            trajectory = self.group_right.plan(goal)

    def plan_cartesian_target(self, pos, quat, arm_name):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]

        if (arm_name == 'left'):
            trajectory = self.group_left.plan(pose_goal)
            self.group_left.go()

        elif (arm_name == 'right'):
            trajectory = self.group_right.plan(pose_goal)
            self.group_right.go()
        elif (arm_name == 'top'):
            trajectory = self.group_top.plan(pose_goal)
            self.group_top.go()
        return trajectory

    def initial_pose(self, arm="all"):
        joint_goal = [0, -0.785, 0, -1.571, 0, 1.571, 0.785]
        if (arm == "left"):
            self.group_left.plan(joint_goal)
            self.group_left.go()
        elif (arm == "right"):
            self.group_right.plan(joint_goal)
            self.group_right.go()
        elif (arm == "top"):
            self.group_top.plan(joint_goal)
            self.group_top.go()
        else:
            for group in self.group_list:
                group.plan(joint_goal)
                group.go()

    def plan_joint_target(self, joint_goal, arm_name='panda_closed_chain'):

        if (len(joint_goal) == 21):
            self.group_123.plan(joint_goal)
            self.group_123.go()

        if (arm_name == 'left'):
            self.group_left.plan(joint_goal)
            self.group_left.go()

        elif (arm_name == 'right'):
            self.group_right.plan(joint_goal)
            self.group_right.go()
        elif (arm_name == 'top'):
            self.group_top.plan(joint_goal)
            self.group_top.go()

    def gripper_open(self, arm="all"):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        if (arm == "left"):
            self.hand_left.plan(joint_goal)
            print("OPEN left GRIPPER")
            self.hand_left.go()
        elif (arm == "top"):
            self.hand_top.plan(joint_goal)
            print("OPEN top GRIPPER")
            self.hand_top.go()
        elif (arm == "right"):
            self.hand_right.plan(joint_goal)
            print("OPEN right GRIPPER")
            self.hand_right.go()
        else:
            for hand in self.hand_list:
                hand.plan(joint_goal)
                hand.go()

    def gripper_close(self):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.0
        trajectory = self.hand_left.plan(joint_goal)
        return trajectory
