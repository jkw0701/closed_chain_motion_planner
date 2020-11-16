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
from suhan_motion_planner import SuhanMotionPlannerManager

import rospkg
import time

class SceneObject():
    def __init__(self):
        rospack = rospkg.RosPack()

        self.stefan_dir = 'package://grasping_point//STEFAN/stl/assembly_without_bottom.stl'
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id = "base"
        # start
        self.assembly_pose.pose.position.x = 0.958 - 0.007
        self.assembly_pose.pose.position.y = 0.213 + 0.01
        self.assembly_pose.pose.position.z = 1.557
        self.assembly_pose.pose.orientation.x =  -0.0446
        self.assembly_pose.pose.orientation.y = 0.0220
        self.assembly_pose.pose.orientation.z = 0.8958
        self.assembly_pose.pose.orientation.w = 0.4418
        
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



if __name__ == '__main__':
    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')

    
    smp = SuhanMotionPlannerManager(sys.argv)
    path_name = "/home/jiyeong/catkin_ws/src/2_social/closed_chain_motion_planner/debug/_path.txt"
    stefan = SceneObject()
    smp.planner.add_collision_mesh(stefan.stefan_dir, np.array([stefan.assembly_pose.pose.position.x, stefan.assembly_pose.pose.position.y, stefan.assembly_pose.pose.position.z]), np.array(
        [stefan.assembly_pose.pose.orientation.x, stefan.assembly_pose.pose.orientation.y, stefan.assembly_pose.pose.orientation.z, stefan.assembly_pose.pose.orientation.w]), "assembly")
    
    path = load_path(path_name)
    # print(path)
    first = True
    for q in path:
        smp.planner.update_arm_states(q)
        smp.planner.publish_planning_scene_msg()
        time.sleep(0.1)
        if (first):
            smp.attach_object("panda_left", "assembly")
            first = False
            time.sleep(0.5)
    
    while rospy.is_shutdown() is False:
        for q in path:
            smp.planner.update_arm_states(q)
            smp.planner.publish_planning_scene_msg()
            time.sleep(0.1)

        
# ##############################################################################################################

