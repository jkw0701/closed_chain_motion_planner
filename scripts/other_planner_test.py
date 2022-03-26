

from __future__ import print_function

import rospy
import moveit_commander
from smach import StateMachine
import smach_ros
import smach

from actionlib import *
from actionlib_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from math import pi
import numpy as np

from assembly_msgs.msg import *
from assembly_msgs.srv import *
import geometry_msgs
from suhan_motion_planner import SuhanMotionPlannerManager 
import time
sm_name = 'joint_move'
sm_loc = '/JOINT_MOVE'

class ClosedChainState(StateMachine):
    def __init__(self, planner):
        self.planner_ = planner

    def get_closed_chain_trajectory(self, arm_names, object_id, desired_pos, desired_quat):
        self.planner_.planner.set_arm_fix('panda_right', True)  
        trajectory = self.planner_.plan_dual_arm_pose(arm_names, object_id, desired_pos, desired_quat)
        if trajectory == None:
            return None

        return_goal = FollowJointTrajectoryGoal()
        return_goal = self.planner_.make_follow_trajectory_goal(trajectory)
        self.planner_.reset_fixed_arm_all()
        return return_goal


def main():
    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('assembly_task_manager', argv=sys.argv, anonymous=True)
    # topic_name = '/assembly/state_transition'
    planner = SuhanMotionPlannerManager(sys.argv)
    

    mesh_path = 'package://closed_chain_motion_planner/stl/dumbbell.stl'
    object_id = 'dumbbell'
    start_pos = np.array([0.65, 0.3, 1.28])
    start_quat = np.array([0, 0, 0, 1])
    planner.last_object_pose[object_id] = (start_pos, start_quat)
    planner.planner.add_collision_mesh(mesh_path,start_pos,start_quat, object_id)
    planner.planner.set_start_object_states(object_id, start_pos, start_quat)
    arm_names = ['panda_left', 'panda_top']
    
    q_init = np.array([-0.001464288952966375, -0.5420558541535008, 0.007754542016750836, -2.7600035963652623, -0.01380276302276182, 2.2305586932141273, 0.7631756594606433,
    0, 0, 0, -pi/3, 0, pi/3, pi/4, 
    -0.006207133778678346, -0.3689234817705707, -0.004603226266084845, -2.6122969356471706, -0.001778466751191603, 2.2477198121699504, 0.7947798967623179])
    planner.planner.set_start_arm_states(q_init)
    planner.planner.update_arm_states(q_init)
    planner.joint_states = q_init

    planner.attach_object('panda_left', object_id)
    
    desired_pos = np.array([0.65, -0.08, 1.28])
    desired_quat = np.array([0, 0, 0, 1])

    sm = ClosedChainState(planner)
    # planner.display_joint_and_env()
    sm.get_closed_chain_trajectory(arm_names, object_id, desired_pos, desired_quat)
    path = planner.dual_arm_task.get_path()
    first = True
    # for q in path:
    #     planner.planner.update_arm_states(q)
    #     planner.joint_states = q
    #     planner.planner.publish_planning_scene_msg()
    #     time.sleep(0.1)
    #     if (first):
    #         planner.planner.publish_planning_scene_msg()
    #         time.sleep(0.5)
    #         planner.attach_object('panda_left', object_id)
    #         first = False
    #         time.sleep(2)

if __name__ == '__main__':
    main()
