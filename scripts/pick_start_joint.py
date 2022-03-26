#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

import yaml
import tf2_ros

from math import radians
from MoveGroupPlanner import MoveGroupPlanner

import rospkg
import argparse


import argparse
import yaml

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--obj_name', default='dumbbell', help='type object name')
args = parser.parse_args()
print(args.obj_name)

class SceneObject():
    def __init__(self):
        rospack = rospkg.RosPack()
        self.mesh_dir = rospack.get_path(
            "closed_chain_motion_planner") + "/stl/" + args.obj_name + ".stl"

        with open(rospack.get_path("closed_chain_motion_planner") + "/config/" + args.obj_name +".yaml", 'r') as file:
            self.yaml = yaml.safe_load(file)

        self.obj_name = self.yaml["obj_name"]
        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "base"

        start_pose = self.yaml["t_wo_start_pos"]
        start_quat = self.yaml["t_wo_start_quat"]
        print(start_pose)
        self.pose.pose.position.x = start_pose[0]
        self.pose.pose.position.y = start_pose[1]
        self.pose.pose.position.z = start_pose[2]
        self.pose.pose.orientation.x =  start_quat[0]
        self.pose.pose.orientation.y = start_quat[1]
        self.pose.pose.orientation.z = start_quat[2]
        self.pose.pose.orientation.w = start_quat[3]


def load_path(path_file):
    tt = []
    path_array = []
    default_right = [0, -0.785, 0, -1.571, 0, 1.571, 0.785]
    with open(path_file, "r") as file:
        for line in file.readlines():
            split_line = line.split(" ")
            if (len(split_line) > 14):
                split_line.pop(-1)
            if (len(split_line) < 14):
                continue
            for i in range(0, 14):
                split_line[i] = float(split_line[i])

            for i in range(0, 7):
                split_line.insert(7+i, default_right[i])

            tt.append(split_line)

    result = np.array(tt)
    return result


def execute_path(mdp, path_name, total_time):

    robot_trajectory = moveit_msgs.msg.RobotTrajectory()
    joint_trajectory = trajectory_msgs.msg.JointTrajectory()
    joint_trajectory.joint_names = mdp.group_123.get_active_joints()
    path = load_path(path_name)
    first = True
    for q in path:
        if first:
            start_joint = q
            first = False
        traj_point = moveit_msgs.msg.trajectory_msgs.msg.JointTrajectoryPoint()
        for i in range(0, 21):
            traj_point.positions.append(q[i])
        joint_trajectory.points.append(traj_point)

    n_traj = len(joint_trajectory.points)
    for i in range(n_traj):
        joint_trajectory.points[i].time_from_start = rospy.Duration(
            total_time / n_traj * i)

    robot_trajectory.joint_trajectory = joint_trajectory

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = mdp.robot.get_current_state()
    display_trajectory.trajectory.append(robot_trajectory)
    return start_joint, robot_trajectory


if __name__ == '__main__':
        sys.argv.append('joint_states:=/panda_dual/joint_states')
        rospy.init_node('ggg')
        r = rospy.Rate(10)

        mdp = MoveGroupPlanner()
        touch_links = mdp.robot.get_link_names(group='hand_triple')

        display_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        while (mdp.display_trajectory_publisher.get_num_connections() == 0 and rospy.is_shutdown is False):
            rospy.spin()

        obj_info = SceneObject()
        rospy.sleep(1)

        mdp.plan_cartesian_target([0.52, 0.3, 1.54], [0.7071, 0.7071, 0, 0], "left")
        mdp.plan_cartesian_target([0.78, 0.3, 1.54], [-0.7071, 0.7071, 0, 0], "top")

        mdp.scene.add_mesh(
            obj_info.obj_name, obj_info.pose, obj_info.mesh_dir)
        rospy.sleep(2)
        rospy.sleep(2)
