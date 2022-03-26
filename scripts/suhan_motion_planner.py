#!/usr/bin/env python

from tf import *
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix, rotation_from_matrix

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from constrained_motion_planning import SuhanMotionPlanner, get_path_from_url, NameVector, NameMap, ParamVector, to_trajectory_msgs, display_from_trajectory_msg
from constrained_motion_planning.suhan_utils import joint_names, display
# from ..params.default_parameters import T_DXL_CTR, T_DRILL0_WO, T_x180_, T_bracket_updater, T_EA_LONG_BOLT_DRILL_RIGHT
import moveit_commander

import pickle
import numpy as np
from numpy import linalg as LA
import rospy
from math import pi
import rospkg
import time
import sys
import copy
import os.path

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from matplotlib import pyplot as plt
from assembly_task_manager import UnlockErrorSM


class SuhanMotionPlannerManager:
    def __init__(self, argv, is_real = False):
        self.working_directory = '/home/dyros/suhan_motion_planning'
        self.debug_file_prefix = self.working_directory + '/logs/'
        self.data_file_prefix = self.working_directory + '/database/'
        self.is_real = is_real # True #to skip 'is_initial_joint_updated' when not using real robot
        self.real_arm_num = 3
        self.arm_num = 3
        moveit_commander.roscpp_initialize(argv)
        self.is_initial_joint_updated = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        self.planner = SuhanMotionPlanner()
        self.planner.set_debug_file_prefix(self.debug_file_prefix)
        self.max_velocity_scaling_factor = 0.2    #0.9      
        self.max_acceleration_scaling_factor = 1.0     #0.35
        self.need_to_update_current_state = True
        self.run_fast = True
        self.obj_list = []
        
        self.joint_states = np.zeros(7*self.arm_num)
        if self.real_arm_num == 2:
            self.joint_states[7*2:7*3] = np.array([0, 0, 0, -pi/3, 0, pi/3, pi/4])
            self.is_initial_joint_updated[7*2:7*3] = [True, True, True, True, True, True, True]
        self.joint_subscriber = rospy.Subscriber('/panda_dual/joint_states', JointState, self.__joint_state_callback)
        # TF
        self.listener = TransformListener()
        self.transformer = TransformerROS()
        self.broadcaster = TransformBroadcaster()
        # Scene Object lists
        # TODO: add objects
        # self.object_data_dict = {'pin_1':[mesh_url, grasp_url, pos, quat],}
        self.object_lists = ['pin_1', 'pin_2']
        self.last_object_pose = {}
        self.arm_names = ['panda_left', 'panda_right', 'panda_top']
        arm = NameMap()
        obj = NameMap()
        arm['panda_left'] = 0
        arm['panda_right'] = 1
        arm['panda_top'] = 2
        obj['dumbbell'] = 0
        self.joint_map = {}
        for i in range(7):
            for name in self.arm_names:
                self.joint_map[name + '_joint' + str(i+1)] = i + 7 * arm[name]
        print (self.joint_map)

        self.hand_touch_links = {}
        self.hand_touch_links['panda_left'] = NameVector()
        self.hand_touch_links['panda_right'] = NameVector()
        self.hand_touch_links['panda_top'] = NameVector()
        self.hand_touch_links['panda_left'].append('panda_left_hand')
        self.hand_touch_links['panda_left'].append('panda_left_leftfinger')
        self.hand_touch_links['panda_left'].append('panda_left_rightfinger')
        self.hand_touch_links['panda_left'].append('panda_left_link7')
        self.hand_touch_links['panda_left'].append('panda_left_link8')
        self.hand_touch_links['panda_right'].append('panda_right_hand')
        self.hand_touch_links['panda_right'].append('panda_right_finger_left_link')
        self.hand_touch_links['panda_right'].append('panda_right_finger_right_link')
        self.hand_touch_links['panda_right'].append('panda_right_link7')
        self.hand_touch_links['panda_right'].append('panda_right_link8')
        self.hand_touch_links['panda_top'].append('panda_top_hand')
        self.hand_touch_links['panda_top'].append('panda_top_finger_left_link')
        self.hand_touch_links['panda_top'].append('panda_top_finger_right_link')
        self.hand_touch_links['panda_top'].append('panda_top_link7')
        self.hand_touch_links['panda_top'].append('panda_top_link8')
        
        self.collision_allowed_ids = NameVector()
        self.collision_allowed_ids.append('ikea_stefan_side_left_0')

        eye = np.array([0,0,0,1])
        reversed_eye = np.array([1,0,0,0])
        q_arm = np.array([0, 0, 0, -pi/3, 0, pi/3, pi/4, 0, 0, 0, -pi/3, 0, pi/3, pi/4, 0, 0, 0, -pi/3, 0, pi/3, pi/4])
        ee_pos = np.array([0, 0, 0.103])
        # ee_pos_R = np.array([0, 0, 0.083])
        ee_pos_R = np.array([0, 0, 0.078]) # for test 11-12 khc
        ee_quat = np.array([0,0,0,1])

        # planner needs to be initialized
        self.planner.set_environment(arm, obj)
        self.planner.set_sigma(0.4)
        self.planner.set_max_ik_trials(10000)
        self.planner.set_max_planning_time(60)
        self.planner.set_sample_rviz(True)
        
        self.planner.set_robot_transform('panda_left', np.array([0, 0.3, 1.006]), eye)
        self.planner.set_robot_transform('panda_right', np.array([0, -0.3, 1.006]), eye)
        self.planner.set_robot_transform('panda_top', np.array([1.35, 0.3, 1.006]), np.array([0,0,1,0]))

        # pos, quat = self.__convert_to_vec_quat(T_DXL_CTR)
        # print (pos, quat)

        self.planner.set_end_effector_frame('panda_left', ee_pos, ee_quat)
        self.planner.set_end_effector_frame('panda_right', ee_pos_R, ee_quat)
        self.planner.set_end_effector_frame('panda_top', ee_pos_R, ee_quat)
        self.planner.set_end_effector_margin('panda_left', np.array([0,0, 0.03]), eye)
        self.planner.set_end_effector_margin('panda_right', np.array([0,0, 0.03]), eye)
        self.planner.set_end_effector_margin('panda_top', np.array([0,0, 0.03]), eye)
        self.long_planner = 'rrt_connect' #
        self.planner.set_chomp_postprocess(False)
        # self.long_planner = 'rrt_connect' #
        # self.planner.set_planner('ait_star')

        # Calib dasta with closed chain calibration with 2.09e-5 eval
        calib_dh_left = np.array([ 
                                [0.000474560429981023,	 0.000483166682165302,	 -0.00304883355188383,	  0.00148667086907321],
                                [-3.69828865924539e-05,	-0.000288069909956647,	 -0.00812428761844092,	  0.00421567136144437],
                                [-0.000154357131719552,	  -0.0010921364777817,	  0.00031894496234845,	  -0.0030474925191138],
                                [-0.000117600404870226,	 0.000712982958085577,	 -0.00571261767823764,	  0.00176867969486185],
                                [-0.00058993701921134,	-0.000326649645213642,	  0.00939394386245098,	  0.00123723772258799],
                                [-0.000433705606644922,	-0.000293762477507038,	  -0.0156742348127345,	 -0.00529320945025931],
                                [-0.000589815315429364,	  6.2389274666678e-05,	   0.0291501803388187,	  0.00113202442328629]])

        calib_dh_right = np.array([ 
                                [-0.00153055068483914,	-0.000772485919009139,	 -0.00374187596482555,	 -0.00183369895482027],
                                [0.000163834922530543,	-0.000212890734425727,	  -0.0041768339596184,	  0.00292223805919776],
                                [-2.60271767179549e-05,	 -0.00100930869860036,	  0.00208915153420725,	 -0.00362801030623702],
                                [0.0002126348171224,	  0.00108679894872676,	 9.81965015663654e-05,	  0.00292912798333623],
                                [-0.00136529072609946,	-5.62799006356164e-05,	   0.0139799357258803,	 -0.00122374898174726],
                                [-0.000502587880406569,	 0.000336192838117574,	  -0.0139808833528828,	 -0.00268675457630875],
                                [-0.00104647166032287,	 0.000135297170834114,	  0.00498364620994882,	 0.000349775306996936]])

        calib_dh_top = np.array([
                                [-0.000171539356569936,	 0.000591551783574421,	  0.00119525002639617,	 -0.00650097874689066],
                                [-0.000330274824097498,	-0.000124214443868683,	 7.10962210595555e-05,	  0.00166835357174836],
                                [-0.00027921463294522,	-0.000683991542242173,	  0.00203760570771006,	  -0.0018819778569208],
                                [-0.000355247972007034,	 0.000891508331427601,	 -0.00513318787489872,	 0.000168196477584221],
                                [0.000190817101859644,	-0.000635118518697479,	  0.00445732420517835,	 -0.00167223312645046],
                                [-0.000244350284995939,	-0.000209543585115151,	 0.000118913154576129,	 -0.00356076901549127],
                                [-0.000484192538997231,	 4.41640702632187e-05,	  -0.0155483388661319,	 0.000602582057747216]])



        self.planner.set_calibration_data('panda_left', calib_dh_left)
        self.planner.set_calibration_data('panda_right', calib_dh_right)
        self.planner.set_calibration_data('panda_top', calib_dh_top)
        # let's add some collision meshes for the objects
        # Note: Use URL (package:// ... )
        # also, grasp candidates should be given.
        # Note: YOU SHOULD USE PATH NOT URL (use get_path_from_url function)
        # TODO: use this

        self.planner.add_box(np.array([0.65, 1.0, 0.2]),'table', np.array([0.65, 0.0, 1.1]), np.array([0, 0, 0, 1]))
        self.obj_list.append('table')
        # self.planner.add_box(np.array([0.5, 0.08, 0.1]),'obstacle', np.array([0.65, 0.1, 1.3]), np.array([0, 0, 0, 1]))
        # self.obj_list.append('obstacle')
        


        # for the first state
        self.planner.set_start_arm_states(q_arm)
        # self.planner.print_start_state()
        # for the visualization
        # q_arm[7:14] = np.array([-0.516055, 0.377285, 0.567762, -2.48491, -2.54167, 1.81389, 2.01595])
        self.planner.update_arm_states(q_arm)
        # print('q_arm: ', q_arm)
        # self.planner.print_current_collision_infos()
        # self.planner.enable_tf_broadcasting()

        self.planning_scene = self.planner.get_planning_scene_collision_check() 
        self.ik_task = self.planner.get_ik_task()
        self.object_ik_task = self.planner.get_object_ik_task()
        self.assembly_ik_task = self.planner.get_assembly_ik_task()
        self.dual_arm_task = self.planner.get_dual_arm_task()
        self.dual_arm_task.set_delta(0.1)
        self.dual_arm_task.set_range(1)

        self.dual_arm_task.set_max_trials(10000)
        self.dual_arm_task.set_cost_limit(100.0)
        self.dual_arm_task.set_enable_sample_viz(False)
        self.triple_ik_task = self.planner.get_triple_ik_task()
        self.orientation_constrained_task = self.planner.get_orientation_constrained_task()

        self.orientation_constrained_task.set_enable_sample_viz(False)
        self.orientation_constrained_task.set_cost_limit(7.0)
        
        self.ik_task.set_grad_rate(5e-2)
        self.ik_task.set_grad_stop_threshold(3e-2)
        self.ik_task.set_max_opt_trials(20)
        self.ik_task.set_cost_limit(5.0)
        self.ik_task.set_enable_sample_viz(True)

        self.object_ik_task.set_grad_rate(5e-2)
        self.object_ik_task.set_grad_stop_threshold(2e-2)
        self.object_ik_task.set_max_trials(10000)
        self.object_ik_task.set_max_opt_trials(20)
        self.object_ik_task.set_cost_limit(7.0)
        self.object_ik_task.set_enable_sample_viz(True)

        self.assembly_ik_task.set_grad_rate(5e-2)
        self.assembly_ik_task.set_grad_stop_threshold(2e-2)
        self.assembly_ik_task.set_max_opt_trials(20)
        self.assembly_ik_task.set_cost_limit(5.0)
        self.assembly_ik_task.set_enable_sample_viz(True)

        self.triple_ik_task.set_grad_rate(5e-2)
        self.triple_ik_task.set_grad_stop_threshold(2e-2)
        self.triple_ik_task.set_max_opt_trials(20)
        self.triple_ik_task.set_cost_limit(8.0)
        self.triple_ik_task.set_enable_sample_viz(True)
        
        self.controller_not_working = False
        
        self.start_q_threshold = 0.087266444 # rad

        if self.is_real:
            is_completed = False
            while is_completed is False and rospy.is_shutdown() is False:
                is_completed = True
                for value in self.is_initial_joint_updated:
                    if value is False:
                        is_completed = False
                print('waiting for joint states. please use SuhanMotionPlannerManager(sys.argv, is_real=False) if you are using this in just sim mode.')
                rospy.sleep(0.1)
        else:
            self.joint_states = np.array([0, 0, 0, -pi/3, 0, pi/3, pi/4,0, 0, 0, -pi/3, 0, pi/3, pi/4,0, 0, 0, -pi/3, 0, pi/3, pi/4])
        # else:
        #     self.joint_states = np.array([])

    def __del__(self):
        self.save_ik_solutions()
        moveit_commander.roscpp_shutdown()

    def __joint_state_callback(self, data):
        for i in range(0,len(joint_names)):
            for j in range(0, len(data.name)):
                if joint_names[i] == data.name[j]:
                    self.joint_states[i] = data.position[j]
                    self.is_initial_joint_updated[i] = True
        # print(self.joint_states)

    def __robot_error_callback(self, subscriber):
        # print("test_subscribe")
        if subscriber.data is True:
          self.controller_not_working = True

    def __update_current_state(self):
        # TODO: implement this code (insepct TF, /panda_dual/joint_states)
        # Noise option (?)
        self.planner.update_arm_states(self.joint_states)
        # self.planner.publish_planning_scene_msg()

        if self.need_to_update_current_state is False:
            return
        self.planner.set_start_arm_states(self.joint_states)
        for col_object in self.object_lists:
            # get obj pose
            # pos = ?
            # quat = ?
            # self.planner.update_object_states(col_object, pos, quat)
            # self.planner.set_start_object_states(col_object, pos, quat)
            pass
        pass
        self.print_current_collision_info()

    def __convert_to_name_vector(self, name_list):
        name_vector = NameVector()
        for name in name_list:
            name_vector.append(name)
        return name_vector

    def __convert_to_param_vector(self, param_list):
        param_vector = ParamVector()
        for param in param_list:
            param_vector.append(param)
        return param_vector

    def __convert_to_vec_quat(self, matrix):
        pos = np.empty(3)
        pos[0:3] = matrix[0:3, 3].T
        quat = quaternion_from_matrix(matrix)
        return pos, quat

    def __load_pickles(self, name):
        if name == None:
            return None
        file_name = self.data_file_prefix + name + '.pkl'
        if os.path.isfile(file_name):
            with open(file_name, 'rb') as f:
                return pickle.load(f)
        else:
            return None

    def __path_to_trajectory(self, path):
        self.planner.set_solved_path(path)
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        self.reset_collision_all_arm()
        return trajectory

    def __process_pickles(self, path):
        if path is None:
            raise RuntimeError('Why did you call me even if the path is empty? Please modify the codes')

        start_q = path[0]
        diff = np.linalg.norm(self.joint_states - start_q)
        print('process_pickle - diff:',diff)
        if (diff < self.start_q_threshold):
            self.planner.add_start_to_path(self.joint_states)
            new_path = np.concatenate(([self.joint_states],path), axis=0)
            return self.__path_to_trajectory(new_path)

        print('the initial q of the given joint path is too far away from the current state')
        print('self.joint_states', self.joint_states)
        print('start_q', start_q)

        print('try to plan')
        for i in range(5):
            print (' - try', i+1)
            r = self.planner.solve_for_joint_pose_all(start_q)
            if r == True:
                print('okay we planner the safe motion to the start')
                path_pre = self.planner.get_solved_path()
                new_path = np.concatenate((path_pre, path), axis=0)

                return self.__path_to_trajectory(new_path)
        
        raise RuntimeError('plan failed (cannot move to saved pickle)')

    def __save_pickles(self, name, data):
        if name == None:
            return

        with open(self.data_file_prefix + name + '.pkl', 'wb') as f:
            pickle.dump(data, f)

    def __check_safe_diff(self, q1, q2):
        d = LA.norm(q1-q2)
        if d > 0.1:
            return False
        return True
    
    def __set_planner_for_coarse_motion(self, name):
        if name is None or self.run_fast:
            self.planner.set_planner('rrt_connect')
        else:
            self.planner.set_planner(self.long_planner) 

    def check_robot_error(self):
        self.controller_not_working = False
        sub_once = None
        sub_once = rospy.Subscriber('/panda_dual/panda_left/has_error', Bool, self.__robot_error_callback, sub_once)
        sub_once = None
        sub_once = rospy.Subscriber('/panda_dual/panda_right/has_error', Bool, self.__robot_error_callback, sub_once)
        sub_once = None
        sub_once = rospy.Subscriber('/panda_dual/panda_top/has_error', Bool, self.__robot_error_callback, sub_once)
        rospy.sleep(0.05)
        # sub_once.unregister()
        if self.controller_not_working:
          print('-------------------------------------------------')
          print('controller not working ... attempting UnlockError')
          print('-------------------------------------------------')
          UnlockErrorSM().execute()
          rospy.sleep(0.7)

    def update_joint_states(self):
        self.planner.update_arm_states(self.joint_states)
        self.planner.set_start_arm_states(self.joint_states)

    def display_joint_and_env(self):
        while rospy.is_shutdown() is False:
            self.planner.update_arm_states(self.joint_states)
            self.planner.set_start_arm_states(self.joint_states)
            self.planner.publish_planning_scene_msg()
            rospy.sleep(0.1)
            
    def display_joint_and_env_given(self, joint):
        while rospy.is_shutdown() is False:
            self.planner.update_arm_states(joint)
            self.planner.set_start_arm_states(joint)
            self.planner.publish_planning_scene_msg()
            rospy.sleep(0.1)

    def set_planning_time_default(self):
        self.planner.set_sigma(0.3)
        self.planner.set_max_ik_trials(20000)
        self.planner.set_max_planning_time(10)

    def disable_collision_with_arm(self, arm_name, object_id, enable):
        for i in range(7):
            self.change_collision('{0}_link{1}'.format(arm_name, i+1), object_id, enable)
        self.change_collision(arm_name + '_hand', object_id, enable)
        if arm_name != 'panda_left':
            self.change_collision(arm_name + '_finger_left_link', object_id, enable)
            self.change_collision(arm_name + '_finger_right_link', object_id, enable)

    def set_collision_arm(self, arm_name):
        not_col_obs = self.planning_scene.get_all_attached_objects()
        for obj in self.obj_list:
            if obj in not_col_obs:
                continue
            for i in range(7):
                self.change_collision('{0}_link{1}'.format(arm_name, i+1), obj, True)
            self.change_collision(arm_name + '_hand', obj, True)
            if arm_name != 'panda_left':
                self.change_collision(arm_name + '_finger_left_link', obj, True)
                self.change_collision(arm_name + '_finger_right_link', obj, True)

    def reset_collision_arm(self,arm_name):
        for obj in self.obj_list:
            for i in range(7):
                self.change_collision('{0}_link{1}'.format(arm_name, i+1), obj, False)
            self.change_collision(arm_name + '_hand', obj, True)
            if arm_name != 'panda_left':
                self.change_collision(arm_name + '_finger_left_link', obj, False)
                self.change_collision(arm_name + '_finger_right_link', obj, False)

    def reset_collision_all_arm(self):
        arm_list = ['panda_left','panda_right','panda_top']
        for arm in arm_list:
            for obj in self.obj_list:
                for i in range(7):
                    self.change_collision('{0}_link{1}'.format(arm, i+1), obj, False)
                self.change_collision(arm + '_hand', obj, False)
                if arm != 'panda_left':
                    self.change_collision(arm + '_finger_left_link', obj, False)
                    self.change_collision(arm + '_finger_right_link', obj, False)

    def set_fixed_arm(self, arm_name):
        self.set_collision_arm(arm_name)
        self.planner.set_arm_fix(arm_name, True)

    def reset_fixed_arm(self, arm_name):
        print ('reset_fixed_arm1')
        self.reset_collision_arm(arm_name)
        self.planner.set_arm_fix(arm_name, False)

    def reset_fixed_arm_all(self):
        self.reset_collision_all_arm()
        self.planner.reset_fix()

    def get_kinematics(self, arm_name):
        self.__update_current_state()
        pos = np.empty(3)
        quat = np.empty(4)
        self.planner.get_arm_ee_pose(arm_name, pos, quat)
        return pos, quat

    # transform w(base) - 0(robot_base) - 7(flange) - e(end_effector) - o(furniture_base)
    def calc_obj_pos(self,arm_name,object_id,grasp_param):
        T_w0_ = np.eye(4)
        if arm_name == 'panda_left':
            T_w0_[0:3,3] = np.array([0.0, 0.3, 1.0])
        elif arm_name == 'panda_right':
            T_w0_[0:3,3] = np.array([0.0, -0.3, 1.0])
        elif arm_name == 'panda_top':
            T_w0_[0,0] = -1.0
            T_w0_[1,1] = -1.0
            T_w0_[0:3,3] = np.array([1.35, 0.3, 1.0])
        p_07_, q_07_ = self.get_kinematics(arm_name)
        T_07_ = quaternion_matrix(q_07_)
        T_07_[0:3,3] = p_07_
        T_7e_ = np.eye(4)
        T_7e_[0:3,3] = np.array([0.0, 0.0, 0.0775])
        p_oe_, q_oe_ = self.get_grasp_pose(object_id, grasp_param)
        T_oe_ = quaternion_matrix(q_oe_)
        T_oe_[0:3,3] = p_oe_
        T_wo_ = np.dot(T_w0_,np.dot(np.dot(T_07_,T_7e_),np.linalg.inv(T_oe_)))
        quat = quaternion_from_matrix(T_wo_)
        pos = translation_from_matrix(T_wo_)
        return pos, quat

    # def calc_obj_pos(self,arm_name,object_id, grasp_index, grasp_param):
    #     T_w0_ = np.eye(4)
    #     if arm_name == 'panda_left':
    #         T_w0_[0:3,3] = np.array([0.0, 0.3, 1.0])
    #     elif arm_name == 'panda_right':
    #         T_w0_[0:3,3] = np.array([0.0, -0.3, 1.0])
    #     elif arm_name == 'panda_top':
    #         T_w0_[0,0] = -1.0
    #         T_w0_[1,1] = -1.0
    #         T_w0_[0:3,3] = np.array([1.35, 0.3, 1.0])
    #     p_07_, q_07_ = self.get_kinematics(arm_name)
    #     T_07_ = quaternion_matrix(q_07_)
    #     T_07_[0:3,3] = p_07_
    #     T_7e_ = np.eye(4)
    #     T_7e_[0:3,3] = np.array([0.0, 0.0, 0.0775])
    #     p_oe_, q_oe_ = self.get_grasp_pose(object_id, grasp_index, grasp_param)
    #     T_oe_ = quaternion_matrix(q_oe_)
    #     T_oe_[0:3,3] = p_oe_
    #     T_wo_ = np.dot(T_w0_,np.dot(np.dot(T_07_,T_7e_),np.linalg.inv(T_oe_)))
    #     quat = quaternion_from_matrix(T_wo_)
    #     pos = translation_from_matrix(T_wo_)
    #     return pos, quat

    # transform w(base) - 0(robot_base) - 7(flange) - e(end_effector) - o(furniture_base)
    def calc_obj_pos_point(self,arm_name, T_oe_):
        T_w0_ = np.eye(4)
        if arm_name == 'panda_left':
            T_w0_[0:3,3] = np.array([0.0, 0.3, 1.0])
        elif arm_name == 'panda_right':
            T_w0_[0:3,3] = np.array([0.0, -0.3, 1.0])
        elif arm_name == 'panda_top':
            T_w0_[0,0] = -1.0
            T_w0_[1,1] = -1.0
            T_w0_[0:3,3] = np.array([1.35, 0.3, 1.0])
        p_07_, q_07_ = self.get_kinematics(arm_name)
        T_07_ = quaternion_matrix(q_07_)
        T_07_[0:3,3] = p_07_
        T_7e_ = np.eye(4)
        T_7e_[0:3,3] = np.array([0.0, 0.0, 0.0825])
        T_wo_ = np.dot(T_w0_,np.dot(np.dot(T_07_,T_7e_),np.linalg.inv(T_oe_)))
        quat = quaternion_from_matrix(T_wo_)
        pos = translation_from_matrix(T_wo_)
        return pos, quat

    def update_object_states(self, object_list):
        for obj in object_list:
            self.update_object_states_each(obj[0], obj[1], obj[2])
            
    def update_box_states(self, box_list):
        for box in box_list:
            self.planner.update_object_states(box[0], box[1], box[2])

    def get_last_object_state(self, object_name):
        return self.last_object_pose[object_name]
    
    def save_ik_solutions(self):
        self.planner.save_ik_solutions()

    def attach_object(self, arm_name, object_id, is_arm=True):
        self.__update_current_state() 
        self.planner.attach_object(object_id, arm_name + '_hand', self.hand_touch_links[arm_name])
        self.planner.publish_planning_scene_msg()

    def detach_object(self, arm_name, object_id):
        self.__update_current_state() 
        self.planner.detach_object(object_id, arm_name + '_hand')

    def remove_collision_object(self, name1):
        self.__update_current_state()
        self.planner.remove_collision_object(name1)

    # change collision state with name1 and name2 / ignore collision when True
    def change_collision(self, name1, name2, allowed):
        # self.__update_current_state()
        self.planner.change_collision(name1, name2, allowed)

    # change collision state with name1 and all the other elements that were used in this function / ignore collision when True
    def change_collisions(self, name1, allowed):
        # self.__update_current_state()
        self.planner.change_collisions(name1, self.collision_allowed_ids, allowed)
        if allowed:
            no_overlap_ = True
            for i in range (len(self.collision_allowed_ids)):
                if self.collision_allowed_ids[i] == name1:
                    no_overlap_ = False
            if no_overlap_: self.collision_allowed_ids.append(name1)
        else: 
            for i in range (len(self.collision_allowed_ids)):
                if self.collision_allowed_ids[i] == name1:
                    self.collision_allowed_ids.erase(self.collision_allowed_ids.begin()+i)
                    break

    # change collision state with name1 and all the other elements / ignore collision when True
    def change_collisions_all(self, name1, allowed):
        self.__update_current_state()
        self.planner.change_collisions_all(name1, allowed)
        if allowed:
            no_overlap_ = True
            for i in range (len(self.collision_allowed_ids)):
                if self.collision_allowed_ids[i] == name1:
                    no_overlap_ = False
            if no_overlap_: self.collision_allowed_ids.append(name1)
        else:
            for i in range (len(self.collision_allowed_ids)):
                if self.collision_allowed_ids[i] == name1:
                    self.collision_allowed_ids.erase(self.collision_allowed_ids.begin()+i)
                    break

    def print_current_collision_info(self):
        self.planner.print_current_collision_infos()

    def make_follow_trajectory_goal(self, trajectory):
        if self.is_real == False:
            display_from_trajectory_msg(self.planner, trajectory, self.joint_states, self.joint_map)
            q = np.array(trajectory.points[-1].positions)
            for i in range(len(trajectory.joint_names)):
                idx = self.joint_map[trajectory.joint_names[i]]
                self.joint_states[idx] = q[i]
        
        return_goal = FollowJointTrajectoryGoal()
        return_goal.trajectory = trajectory
        return return_goal

    def plan_joint_pose(self, arm_name, joint_pose, name = None):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            return self.__process_pickles(path)
            # self.planner.set_solved_path(path)
            # self.planner.add_start_to_path(self.joint_states)
            # self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            # trajectory = to_trajectory_msgs(self.planner)
            # self.reset_collision_all_arm()
            # return trajectory
        self.__update_current_state() 
        self.__set_planner_for_coarse_motion(name)
        r = self.planner.solve_for_joint_pose(arm_name, joint_pose)
        if r == False:
            raise RuntimeError('plan failed')
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        
        path = self.planner.get_solved_path()
        self.planner.update_arm_states(path[-1])
        self.planner.set_start_arm_states(path[-1])

        trajectory = to_trajectory_msgs(self.planner)

        self.__save_pickles(name, path)
        self.reset_collision_all_arm()
        return trajectory

    def plan_given_path(self, path, name = None):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            return self.__process_pickles(path)
            # self.planner.set_solved_path(path)
            # self.planner.add_start_to_path(self.joint_states)
            # self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            # trajectory = to_trajectory_msgs(self.planner)
            # return trajectory

        self.planner.set_solved_path(path)
        self.planner.add_start_to_path(self.joint_states)
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        self.__save_pickles(name, path)
        return trajectory

    def plan_pickle(self, name):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            return self.__process_pickles(path)
            # self.planner.set_solved_path(path)
            # self.planner.add_start_to_path(self.joint_states)
            # self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            # trajectory = to_trajectory_msgs(self.planner)
            # return trajectory
            
        raise RuntimeError('plan failed')

    def plan_target_pose(self, arm_name, target_pos, target_quat, name = None):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            return self.__process_pickles(path)
            # self.planner.set_solved_path(path)
            # self.planner.add_start_to_path(self.joint_states)
            # self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            # trajectory = to_trajectory_msgs(self.planner)
            # self.reset_collision_all_arm()
            # return trajectory

        self.__update_current_state()
        self.__set_planner_for_coarse_motion(name)
        r = self.planner.solve_for_ik_pose(arm_name, target_pos, target_quat)
        if r == False:
            raise RuntimeError('plan failed')
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)

        path = self.planner.get_solved_path()
        self.__save_pickles(name, path)
        self.reset_collision_all_arm()
        return trajectory


    # CAUTION: this resets all fixed arms
    def plan_dual_arm_pose(self, arm_names, object_id, desired_pos, desired_quat, name = None):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            return self.__process_pickles(path)
            # self.planner.set_solved_path(path)
            # self.planner.add_start_to_path(self.joint_states)
            # self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            # trajectory = to_trajectory_msgs(self.planner)
            # self.reset_collision_all_arm()
            # return trajectory
        self.__update_current_state()
        pos, quat = self.last_object_pose[object_id]
        
        self.dual_arm_task.set_arm_names(self.__convert_to_name_vector(arm_names))
        print ('joints:', self.joint_states)
        self.dual_arm_task.set_start_and_goal(self.joint_states, pos, quat, desired_pos, desired_quat)
        self.dual_arm_task.set_timeout_limit(120.0)
        r = self.dual_arm_task.solve()
        
        if r == False:
            raise RuntimeError('plan failed')
        
        path = self.dual_arm_task.get_path()
        self.planner.set_solved_path(path)
        self.planner.time_parameterize(0.5, 0.5)

        # for dual traj
        print (arm_names)
        for arm in self.arm_names:
            if arm in arm_names:
                continue
            self.planner.set_arm_fix(arm, True)
            print ('fix', arm)
        trajectory = to_trajectory_msgs(self.planner)
        # reset
        self.planner.reset_fix()
        self.__save_pickles(name, path)
        self.reset_collision_all_arm()
        return trajectory

    # CAUTION: this resets all fixed arms
    def plan_dual_arm_pose2(self, arm_names, object_id, desired_pos, desired_quat, name = None):
        if self.is_real: self.check_robot_error()
        self.__update_current_state()
        path = self.__load_pickles(name)
        if path is not None:
            self.planner.set_solved_path(path)
            self.planner.add_start_to_path(self.joint_states)
            self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
            trajectory = to_trajectory_msgs(self.planner)
            self.reset_collision_all_arm()
            return trajectory

        pos, quat = self.last_object_pose[object_id]
        
        self.dual_arm_task.set_arm_names(self.__convert_to_name_vector(arm_names))
        self.dual_arm_task.set_start_and_goal(self.joint_states, pos, quat, desired_pos, desired_quat)
        r = self.dual_arm_task.solve2()
        
        if r == False:
            raise RuntimeError('plan failed')
        
        path = self.dual_arm_task.get_path()
        self.planner.set_solved_path(path)
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)

        # for dual traj
        for arm in arm_names:
            if arm in self.arm_names:
                continue
            self.planner.set_arm_fix(arm, True)
        trajectory = to_trajectory_msgs(self.planner)
        # reset
        self.planner.reset_fix()
        self.__save_pickles(name, path)
        self.reset_collision_all_arm()
        return trajectory

   


    def display_path(self, dt = 0.01):
        display(self.planner,dt)

    def get_moved_joint_pose(self, arm_name, pos, quat, q0):
        q_out = np.zeros(7)
        suc = self.planner.get_moved_joints(arm_name, pos, quat, q0, q_out)
        print('q0', q0)
        print('q_', q_out)
        if suc is False:
            print("Fail")
            return None
        else:
            return q_out

    def reproject_path(self, path_file, tol, iteration, idx, offset):
        tt = []
        with open(path_file, "r") as file:
            for line in file.readlines():
                split_line = line.split(" ")
                if (len(split_line) > 21):
                    split_line.pop(-1)
                if (len(split_line) < 21):
                    continue
                for i in range(21):
                    split_line[i] = float(split_line[i])
                tt.append(split_line)

        path = np.array(tt)
        self.planner.set_solved_path(path)
        self.__update_current_state() 
        print(offset)
        success = self.planner.reproject_path(self.joint_states, tol, iteration, idx, offset)
        if success is True:
            print("PROJECTION SUCCESS!!")
        else:
            print("PROJECTION FAILED")
        
            
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        return trajectory



if __name__ == "__main__":
    # TODO: Change the test codes using SuhanMotionPlannerManager
    rospy.init_node('suhan_motion_planner', anonymous=True)
    smpm = SuhanMotionPlannerManager(sys.argv)

    # rospy.spin()
    while smpm.joint_subscriber.get_num_connections() == 0:
        smpm.planner.publish_planning_scene_msg()
        rospy.sleep(0.5)
        
    smpm.planner.set_start_arm_states(smpm.joint_states)
    smpm.planner.update_arm_states(smpm.joint_states)
    smpm.planner.publish_planning_scene_msg()

    traj = smpm.plan_object_ik('panda_right','long_part', 0.32)
    if traj == None:
        print('plan failed')
        exit()

    client = actionlib.SimpleActionClient('/assembly_dual_controller/joint_trajectory_control', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    
    client.send_goal(goal)
    client.wait_for_result()

    print(client.get_result())
