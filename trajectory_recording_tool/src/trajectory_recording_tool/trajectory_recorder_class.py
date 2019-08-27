"""
# Name: trajectory_recorder_class.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 02/07/2018
# Edit Date: 03/20/2018
#
# Description:
# Class to
# - record a series of trajectory waypoints with user-specified times/velocities.
# - save the waypoints to a [ROS_PACKAGE]/recorded_trajectories/[NAME].csv
#   where [ROS_PACKAGE] and [NAME] are specified by the user.
#
# Generated trajectory *.json files can be loaded into a generic
# ExecuteJointTrajectory SMACH state instances to compose state machine behaviors.
"""

from __future__ import print_function

import sys
import os
import json
import re
import math as m
import xml.etree.ElementTree as ET
import numpy as np
from threading import Thread
import copy

from tabulate import tabulate
import rospy
from rospkg import RosPack, ResourceNotFound
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from actionlib.simple_action_client import SimpleActionClient
from urdf_parser_py.urdf import URDF
from hebiros.srv import AddGroupFromNamesSrv, SetCommandLifetimeSrv
from hebiros.msg import TrajectoryAction, TrajectoryGoal, WaypointMsg
from pykdl_utils.kdl_kinematics import KDLKinematics
from trac_ik_python.trac_ik import IK

from waypoint_class import Waypoint
from rviz_interactive_markers import InteractiveMarkerManager
from common import transformations as transforms  # Note: Avoid collision with ROS tf / tf2
sys.stdout.flush()

NAN = float('nan')


class TrajectoryRecorder(object):
    """
    Attributes
        hebi_group_name     (str):
        hebi_families       (list of str): HEBI actuator families [base,..., tip]
        hebi_names          (list of str): HEBI actuator names [base,..., tip]
    """
    def __init__(self, hebi_group_name, hebi_families, hebi_names):
        rospy.loginfo("Creating TrajectoryRecorder instance...")
        rospy.loginfo("  hebi_group_name: %s", hebi_group_name)
        rospy.loginfo("  hebi_families: %s", hebi_families)
        rospy.loginfo("  hebi_names: %s", hebi_names)
        self.hebi_group_name = hebi_group_name
        self.hebi_families = hebi_families
        self.hebi_names = hebi_names
        self.hebi_mapping = [family+'/'+name for family,name in zip(hebi_families,hebi_names)]

        # jt information populated by self._feedback_cb
        self.current_jt_pos = {}
        self.current_jt_vel = {}
        self.current_jt_eff = {}
        self._joint_state_pub = None

        # Create a service client to create a group
        set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        # Create a service client to set the command lifetime
        self.set_command_lifetime = rospy.ServiceProxy("/hebiros/"+hebi_group_name+"/set_command_lifetime",
                                                       SetCommandLifetimeSrv)
        # Topic to receive feedback from a group
        self.hebi_group_feedback_topic = "/hebiros/"+hebi_group_name+"/feedback/joint_state"
        rospy.loginfo("  hebi_group_feedback_topic: %s", "/hebiros/"+hebi_group_name+"/feedback/joint_state")
        # Topic to send commands to a group
        self.hebi_group_command_topic = "/hebiros/"+hebi_group_name+"/command/joint_state"
        rospy.loginfo("  hebi_group_command_topic: %s", "/hebiros/"+hebi_group_name+"/command/joint_state")
        # Call the /hebiros/add_group_from_names service to create a group
        rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
        rospy.wait_for_service('/hebiros/add_group_from_names')  # block until service server starts
        rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
        set_hebi_group(hebi_group_name, hebi_names, hebi_families)
        # Feedback/Command
        self.fbk_sub = rospy.Subscriber(self.hebi_group_feedback_topic, JointState, self._feedback_cb)
        self.cmd_pub = rospy.Publisher(self.hebi_group_command_topic, JointState, queue_size=1)
        self._hold_position = False
        self._hold_joint_states = []
        # TrajectoryAction client
        self.trajectory_action_client = SimpleActionClient("/hebiros/"+hebi_group_name+"/trajectory", TrajectoryAction)
        rospy.loginfo("  Waiting for TrajectoryActionServer at %s ...", "/hebiros/"+hebi_group_name+"/trajectory")
        self.trajectory_action_client.wait_for_server()  # block until action server starts
        self._executing_trajectory = False
        rospy.loginfo("  TrajectoryActionServer AVAILABLE.")

        # Check ROS Parameter server for robot_description URDF
        urdf_str = ""
        urdf_loaded = False
        time_end_check = rospy.Time.now().to_sec() + 2.0  # Stop looking for URDF after 2 seconds of ROS time
        while not rospy.is_shutdown() and not urdf_loaded and rospy.Time.now().to_sec() < time_end_check:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                urdf_loaded = True
                rospy.loginfo("Pulled /robot_description from parameter server.")
            else:
                rospy.sleep(0.05)  # sleep for 50 ms of ROS time

        if urdf_loaded:
            # pykdl_utils setup
            self.robot_urdf = URDF.from_xml_string(urdf_str)
            rospy.loginfo("URDF links: " + str([link.name for link in self.robot_urdf.links])[1:-1])
            rospy.loginfo("URDF joints: " + str([joint.name for joint in self.robot_urdf.joints])[1:-1])
            valid_names = False
            while not rospy.is_shutdown() and not valid_names:
                # self.chain_base_link_name = self.raw_input_ros_safe("Please enter kinematics chain's base link name\n")  # FIXME: TEMP
                # self.chain_end_link_name = self.raw_input_ros_safe("Please enter kinematics chain's end link name\n")    # FIXME: TEMP
                self.chain_base_link_name = "a_2043_01_5Z"  # FIXME: TEMP
                self.chain_end_link_name = "a_2039_02_4Z"   # FIXME: TEMP
                try:
                    self.kdl_kin = KDLKinematics(self.robot_urdf, self.chain_base_link_name, self.chain_end_link_name)
                    valid_names = True
                except KeyError:
                    rospy.loginfo("Incorrect base or end link name. Please try again...")
            # trac-ik setup
            ik_solver = IK(self.chain_base_link_name,
                           self.chain_end_link_name,
                           urdf_string=urdf_str,
                           timeout=0.01,
                           epsilon=1e-4,
                           solve_type="Distance")
            # Determine transformation from global reference frame to base_link
            urdf_root = ET.fromstring(urdf_str)
            cadassembly_metrics = urdf_root.find('link/CyPhy2CAD/CADAssembly_metrics')
            if cadassembly_metrics is not None:
                robot_base_link_transform = np.zeros((4,4))
                robot_base_link_transform[3,3] = 1
                rotation_matrix_elem = cadassembly_metrics.find('RotationMatrix')
                for j, row in enumerate(rotation_matrix_elem.iter('Row')):
                    for i, column in enumerate(row.iter('Column')):
                        robot_base_link_transform[j,i] = column.get('Value')
                translation_elem = cadassembly_metrics.find('Translation')
                for j, attribute in enumerate(['X','Y','Z']):
                    robot_base_link_transform[j,3] = translation_elem.get(attribute)
                kdl_kin_robot_base_link_to_chain_base_link = KDLKinematics(self.robot_urdf, 'base_link', self.chain_base_link_name)
                jt_angles = [0.0]*kdl_kin_robot_base_link_to_chain_base_link.num_joints
                chain_base_link_transform = kdl_kin_robot_base_link_to_chain_base_link.forward(jt_angles)
                self.chain_base_link_abs_transform = np.dot(robot_base_link_transform,chain_base_link_transform)
            else:
                self.chain_base_link_abs_transform = None
            # Wait for connections to be setup
            while not rospy.is_shutdown() and len(self.current_jt_pos) < len(self.hebi_mapping):
                rospy.sleep(0.1)
            # Set up joint state publisher
            self._joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
            self._active_joints = self.kdl_kin.get_joint_names()
            # Set up RViz Interactive Markers
            self.int_markers_man = InteractiveMarkerManager(self, 'trajectory_recording_tool/interactive_markers', ik_solver=ik_solver)
        else:
            self.robot_urdf = None

        self.waypoints = []
        self.waypoints_cnt = 0
        self.breakpoints_cnt = 0
        self._prev_breakpoint = True

        # rospkg
        self._rospack = RosPack()
        print()

    def set_waypoint(self):
        waypoint = Waypoint()
        joint_state = waypoint.joint_state
        joint_state.names = self.hebi_mapping
        joint_state.positions = self._get_joint_angles()
        if self.robot_urdf is not None:
            homogeneous_transform = self.kdl_kin.forward(joint_state.positions)
            waypoint.end_effector_pose = self._get_pose_from_homogeneous_matrix(homogeneous_transform)
        print("Store these joint positions as Waypoint #{}?:".format(self.waypoints_cnt+1))
        for name,pos in zip(self.hebi_mapping,joint_state.positions):
            print("  {:20}: {:4f}".format(name,pos))
        user_input = self.raw_input_ros_safe("[Return] - yes | [Any other key] - no\n")
        if user_input != "" and user_input != " ":
            self.release_position()
            return False
        self.hold_position()

        valid_input = False
        print("\nPlease enter velocities (optional) for Waypoint #{} in the following format:".format(self.waypoints_cnt+1))
        print("Ex: 0,none,0")
        user_input = self.raw_input_ros_safe("[Return] - skip | velocity1,velocity2, ...velocityN\n")
        while not rospy.is_shutdown() and not valid_input:
            if user_input == "" or user_input == " ":
                joint_state.velocities = [NAN]*len(self.hebi_mapping)
                valid_input = True
            elif len(user_input.split(",")) != len(self.hebi_mapping):
                print("  INVALID INPUT: velocity list must be the same size as module list")
                print("  Please try again.")
                user_input = self.raw_input_ros_safe("[Return] - skip | velocity1,velocity2, ...velocityN\n")
            else:
                joint_state.velocities = [NAN if vel.strip().lower() == 'none' else float(vel) for vel in user_input.split(",")]
                valid_input = True

        valid_input = False
        print("\nPlease enter accelerations (optional) for Waypoint #{} in the following format:".format(self.waypoints_cnt+1))
        print("Ex: 0,1.1,none")
        user_input = self.raw_input_ros_safe("[Return] - skip | accelerations1,accelerations2, ...accelerationsN\n")
        while not rospy.is_shutdown() and not valid_input:
            if user_input == "" or user_input == " ":
                joint_state.accelerations = [NAN]*len(self.hebi_mapping)
                valid_input = True
            elif len(user_input.split(",")) != len(self.hebi_mapping):
                print("  INVALID INPUT: acceleration list must be the same size as module list")
                print("  Please try again.")
                user_input = self.raw_input_ros_safe("[Return] - skip | accelerations1,accelerations2, ... accelerationsN\n")
            else:
                joint_state.accelerations = [NAN if acc.strip().lower() == "none" else float(acc) for acc in user_input.split(",")]
                valid_input = True

        user_input = self.raw_input_ros_safe("\nPlease enter a duration [s] - time to move from the previous waypoint to the current waypoint:\n")
        while not rospy.is_shutdown() and (user_input == "" or user_input == " "):
            user_input = self.raw_input_ros_safe("Please enter a duration [s] - time to move from the previous waypoint to the current waypoint:\n")
        waypoint.time = float(user_input)

        self.append_waypoint(waypoint)
        print("\nWaypoint #{} stored!\n".format(self.waypoints_cnt))
        self.release_position()
        return True

    @staticmethod
    def raw_input_ros_safe(prompt=None):
        sys.stdout.flush()
        try:
            if prompt is not None:
                user_input = raw_input(prompt)
            else:
                user_input = raw_input()
            sys.stdout.flush()
            return user_input
        except KeyboardInterrupt:
            sys.exit()

    def append_waypoint(self, waypoint):
        self.waypoints.append(waypoint)
        self.waypoints_cnt += 1
        self.int_markers_man.add_waypoint_marker(waypoint, str(self.waypoints_cnt))
        self._prev_breakpoint = False

    def append_breakpoint(self):
        if self._prev_breakpoint:
            print("Error: Cannot set two consecutive breakpoints!")
        else:
            self._prev_breakpoint = True
            self.waypoints.append(None)
            self.breakpoints_cnt += 1
            print("\nBreakpoint #{} stored!\n".format(self.breakpoints_cnt))

    def record_movement(self):
        resolution_xyz = None
        if self.robot_urdf is not None:
            resolution_xyz = 0.01*float(self.raw_input_ros_safe("Please enter the desired end-effector Cartesian resolution [cm]:\n"))  # TODO: Add some user-input checking functions
        resolution_jt = (m.pi/180)*float(self.raw_input_ros_safe("Please enter the desired joint resolution [degrees]:\n"))
        duration = float(self.raw_input_ros_safe("Please enter a setup duration for this movement [s]:\n"))

        # Set up Thread
        user_input = [None]
        t = Thread(target=self._wait_for_user_input, args=(user_input,))
        t.start()

        # first post
        prev_jt_angles = self._get_joint_angles()
        prev_end_effector_xyz = None
        waypoint = Waypoint()
        joint_state = waypoint.joint_state
        joint_state.names = self.hebi_mapping
        joint_state.positions = prev_jt_angles
        joint_state.velocities = self._get_joint_velocities()
        joint_state.accelerations = [NAN]*len(self.hebi_mapping)  # TODO: Can we get these from anywhere?
        if self.robot_urdf is not None:
            homogeneous_transform = self.kdl_kin.forward(joint_state.positions)
            waypoint.end_effector_pose = self._get_pose_from_homogeneous_matrix(homogeneous_transform)
            eff_pos = waypoint.end_effector_pose.position
            prev_end_effector_xyz = [eff_pos.x, eff_pos.y, eff_pos.z]
        waypoint.time = duration
        self.append_waypoint(waypoint)
        prev_time = rospy.Time.now().to_sec()

        # subsequent posts
        while not rospy.is_shutdown() and user_input[0] is None:
            jt_angles = self._get_joint_angles()
            end_effector_pose = None
            end_effector_xyz = None
            xyz_dist = None
            if self.robot_urdf is not None:
                homogeneous_transform = self.kdl_kin.forward(joint_state.positions)
                end_effector_pose = self._get_pose_from_homogeneous_matrix(homogeneous_transform)
                eff_pos = waypoint.end_effector_pose.position
                end_effector_xyz = [eff_pos.x, eff_pos.y, eff_pos.z]
                xyz_dist = self._get_abs_distance(end_effector_xyz, prev_end_effector_xyz)
            jt_dist = self._get_abs_distance(jt_angles, prev_jt_angles)
            if (self.robot_urdf is not None and xyz_dist > resolution_xyz) or jt_dist > resolution_jt:
                cur_time = rospy.Time.now().to_sec()
                waypoint = Waypoint()
                joint_state = waypoint.joint_state
                joint_state.positions = jt_angles
                joint_state.velocities = self._get_joint_velocities()
                joint_state.accelerations = [NAN]*len(self.hebi_mapping)  # TODO: Can we get these from anywhere?
                waypoint.end_effector_pose = end_effector_pose
                waypoint.time = cur_time - prev_time
                self.append_waypoint(waypoint)
                prev_jt_angles = jt_angles
                prev_end_effector_xyz = end_effector_xyz
                prev_time = cur_time
        t.join()

    @staticmethod
    def _get_abs_distance(vector1, vector2):
        assert len(vector1) == len(vector2)
        sum_of_squares = 0.0
        for i, (val1, val2) in enumerate(zip(vector1, vector2)):
            sum_of_squares += (val1-val2)**2
        return m.sqrt(sum_of_squares)

    @staticmethod
    def _wait_for_user_input(user_input):
        user_input[0] = raw_input("Press [Return] to stop recording!!!\n")

    @staticmethod
    def _get_pose_from_homogeneous_matrix(homogeneous_transform_matrix):
        pose = Pose()
        pose.position.x = round(homogeneous_transform_matrix[0, 3], 6)
        pose.position.y = round(homogeneous_transform_matrix[1, 3], 6)
        pose.position.z = round(homogeneous_transform_matrix[2, 3], 6)
        quaternion = transforms.quaternion_from_matrix(homogeneous_transform_matrix[:3, :3])  # TODO: Check me
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        return pose

    def print_waypoints(self):
        breakpoints_passed = 0
        time_from_start = 0.0
        for i, waypoint in enumerate(self.waypoints):
            if waypoint is not None:
                time_from_start += waypoint.time
                print("\nWaypoint #{} at time {} [s] from trajectory start".format(i+1-breakpoints_passed,time_from_start))
                joint_state = waypoint.joint_state
                table = [[name,pos,vel,acc]
                         for name,pos,vel,acc
                         in zip(joint_state.names,joint_state.positions,joint_state.velocities,joint_state.accelerations)]
                print(tabulate(table, headers=['Names', 'Positions [rad]', 'Velocities [rad/s]', 'Accelerations [rad/s^2]']))
                eff_position = waypoint.end_effector_pose.position
                print("\nEnd effector position x={}, y={}, z={}".format(eff_position.x, eff_position.y, eff_position.z))
                print("-"*(len("\nWaypoint #{} at time {} [s] from trajectory start")-4+len(str(i)+str(time_from_start))))
            else:
                breakpoints_passed += 1
                print("\nBreakpoint #{}".format(breakpoints_passed))
                print(("-"*(len("\nBreakpoint #{}")-4+len(str(i)))))

    def execute_trajectories(self):
        trajectory_goals, tmp = self._get_trajectory_and_end_effector_goals_from_waypoints()
        # execute initial position-to-start trajectory
        if len(trajectory_goals) == 0:
            print("Error: No trajectory goals to execute!")
        else:
            # execute initial position to start trajectory
            init_goal = TrajectoryGoal()
            waypoint_1 = WaypointMsg()
            waypoint_1.names = self.hebi_mapping
            waypoint_1.positions = self._get_joint_angles()
            waypoint_1.velocities = [0]*len(self.hebi_mapping)
            waypoint_1.accelerations = [0]*len(self.hebi_mapping)
            waypoint_2 = trajectory_goals[0].waypoints[0]
            init_goal.waypoints = [waypoint_1, waypoint_2]
            init_goal.times = [0, 3]
            self._executing_trajectory = True
            self.trajectory_action_client.send_goal(init_goal)
            self.trajectory_action_client.wait_for_result()
            # execute trajectory goals
            for goal in trajectory_goals:
                self.trajectory_action_client.send_goal(goal)
                self.trajectory_action_client.wait_for_result()
                self.trajectory_action_client.get_result()
            self._executing_trajectory = False

    def execute_trajectory(self):
        # TODO: Maybe support executing individual trajectories. Project creep... this is just a command line tool...
        pass

    def save_trajectory_to_file(self):
        # Get target package path
        rospack = RosPack()
        target_package_found = False
        package_path = None
        while not target_package_found:
            target_package_name = raw_input("Please enter target package: ")
            try:
                package_path = rospack.get_path(target_package_name)
                target_package_found = True
                print("Target package path: {}".format(package_path))
            except ResourceNotFound:
                print("Package {} not found!!! Please try again.".format(target_package_name))
        # Create trajectories directory if it does not exist - https://stackoverflow.com/a/14364249
        save_dir_path = os.path.join(package_path, "trajectories")
        print("Save directory path: {}".format(save_dir_path))
        try:  # prevents a common race condition - duplicated attempt to create directory
            os.makedirs(save_dir_path)
        except OSError:
            if not os.path.isdir(save_dir_path):
                raise
        # Get save file name
        name_set = False
        base_filename = None
        while not name_set:
            base_filename = raw_input("Please enter the save file name: ")
            target_name_path_1 = os.path.join(save_dir_path, base_filename+".json")
            target_name_path_2 = os.path.join(save_dir_path, base_filename+"_0.json")
            if os.path.isfile(target_name_path_1) or os.path.isfile(target_name_path_2):
                print("A file with name {} already exists!!! Please try again.".format(base_filename))
            else:
                name_set = True

        # Dump trajectory artifacts to file(s)
        trajectory_goals, end_effector_goals = self._get_trajectory_and_end_effector_goals_from_waypoints()
        for i,(trajectory_goal,end_effector_goal) in enumerate(zip(trajectory_goals,end_effector_goals)):
            suffix ="_"+str(i)
            path = os.path.join(save_dir_path, base_filename+suffix+".json")
            print("Saving trajectory {} to: {}\n".format(i, path))
            with open(path, 'w') as savefile:
                json_data_structure = self._convert_trajectory_goal_to_json_serializable(trajectory_goal, end_effector_goal)
                json_str = json.dumps(json_data_structure, sort_keys=True, indent=4, separators=(',', ': '))
                match_re = r'(\n*\s*\[)(\n\s*("*[\w\./\-\d]+"*,*\n\s*)+\])'
                reformatted_json_str = re.sub(match_re, self._newlinereplace, json_str)
                savefile.write(reformatted_json_str)

    def _convert_trajectory_goal_to_json_serializable(self, trajectory_goal, end_effector_goal):
        waypoints_dict = {}
        for i, (waypoint, pose) in enumerate(zip(trajectory_goal.waypoints, end_effector_goal)):
            waypoint_dict = {
                'names': list(waypoint.names),
                'positions': [float(positions) for positions in waypoint.positions],
                'velocities': [float(velocities) for velocities in waypoint.velocities],
                'accelerations': [float(acceleration) for acceleration in waypoint.accelerations]
            }
            if self.robot_urdf is not None:
                eff_position = pose.position
                waypoint_dict['end-effector xyz'] = [eff_position.x, eff_position.y, eff_position.z]
                eff_orientation = pose.orientation
                waypoint_dict['end-effector wxyz'] = [eff_orientation.w, eff_orientation.x, eff_orientation.y, eff_orientation.z]
            waypoints_dict[str(i)] = waypoint_dict
        times_list = [float(time) for time in trajectory_goal.times]

        json_data_structure = {
            'waypoints': waypoints_dict,
            'times': times_list
        }
        if self.chain_base_link_abs_transform is not None:
            json_data_structure['base link transform'] = self.chain_base_link_abs_transform.tolist()
        return json_data_structure

    @staticmethod
    def _newlinereplace(matchobj):
        no_newlines = matchobj.group(2).replace("\n","")  # eliminate newline characters
        no_newlines = no_newlines.split()  # eliminate excess whitespace
        no_newlines = "".join([" " + segment for segment in no_newlines])
        return matchobj.group(1) + no_newlines

    def _get_trajectory_and_end_effector_goals_from_waypoints(self):
        trajectory_goals = []
        end_effector_goals = []
        prev_breakpoint = False
        prev_time = 0.0
        for waypoint in self.waypoints:

            if waypoint is not None:
                if len(trajectory_goals) == 0 or prev_breakpoint:
                    trajectory_goals.append(TrajectoryGoal())
                    end_effector_goals.append([])
                # if applicable, 1st append last waypoint from previous trajectory
                if len(trajectory_goals) > 1 and prev_breakpoint:
                    waypoint_msg = WaypointMsg()
                    waypoint_msg.names = list(trajectory_goals[-2].waypoints[-1].names)
                    waypoint_msg.velocities = [0]*len(waypoint_msg.names)
                    waypoint_msg.accelerations = [0]*len(waypoint_msg.names)
                    trajectory_goals[-1].waypoints.append(waypoint_msg)
                    trajectory_goals[-1].times.append(0.0)
                    end_effector_goals[-1].append(copy.deepcopy(end_effector_goals[-2]))
                # append a new waypoint
                trajectory_goals[-1].waypoints.append(copy.deepcopy(waypoint.joint_state))
                prev_time += waypoint.time
                trajectory_goals[-1].times.append(prev_time)
                end_effector_goals[-1].append(copy.deepcopy(waypoint.end_effector_pose))
                prev_breakpoint = False

            else:  # breakpoint
                prev_time = 0.0
                prev_breakpoint = True

        return trajectory_goals, end_effector_goals

    def delete_waypoint(self, waypoint):
        index = self.waypoints.index(waypoint)
        # Get index of last waypoint
        deleting_last_wp = False
        for i, wp in reversed(list(enumerate(self.waypoints))):
            if wp is not None:
                if i == index:
                    deleting_last_wp = True
                break
        self.waypoints.remove(waypoint)
        self.waypoints_cnt -= 1
        last_wp = None
        # Check for and remove adjacent breakpoints. Also record last waypoint
        prev_wp = None
        for i, wp in enumerate(list(self.waypoints)):
            if wp is None and prev_wp is None:
                del self.waypoints[i]
                self.breakpoints_cnt -= 1
            else:
                last_wp = wp
            prev_wp = wp
        # Check if last waypoint is a breakpoint
        if len(self.waypoints) == 0 or self.waypoints[-1] is None:
            self._prev_breakpoint = True
        print("Waypoint deleted...")

        if len(self.waypoints) != 0 and index == 0:
            self.waypoints[0].joint_state.velocities = [0]*len(self.hebi_mapping)
            self.waypoints[0].joint_state.accelerations = [0]*len(self.hebi_mapping)
            self.waypoints[0].time = 0
            # TODO: Fix time. Have to access waypoint marker menus, etc...
        elif deleting_last_wp and last_wp is not None:
            i = self.waypoints.index(last_wp)
            self.waypoints[i].joint_state.velocities = [0]*len(self.hebi_mapping)
            self.waypoints[i].joint_state.accelerations = [0]*len(self.hebi_mapping)

    def insert_waypoint(self, ref_waypoint, before=True):
        # create new Waypoint
        waypoint = Waypoint()
        waypoint.joint_state.names = self.hebi_mapping
        waypoint.joint_state.positions = self._get_joint_angles()
        waypoint.joint_state.velocities = [NAN]*len(self.hebi_mapping)
        waypoint.joint_state.accelerations = [NAN]*len(self.hebi_mapping)
        if self.robot_urdf is not None:
            homogeneous_transform = self.kdl_kin.forward(waypoint.joint_state.positions)
            waypoint.end_effector_pose = self._get_pose_from_homogeneous_matrix(homogeneous_transform)
        waypoint.time = 2.0  # TODO: Figure out the best way to edit this from GUI
        # determine index of reference waypoint
        ref_index = None
        wp_passed_cnt = 0
        for i, wp in enumerate(self.waypoints):
            if wp is ref_waypoint:
                ref_index = i
                break
            elif wp is not None:
                wp_passed_cnt +=1
        # insert new Waypoint
        if before:
            if ref_index == 0:
                # undo previous initial waypoint settings
                self.waypoints[0].joint_state.velocities = [NAN]*len(self.hebi_mapping)
                self.waypoints[0].joint_state.accelerations = [NAN]*len(self.hebi_mapping)
                self.waypoints[0].time = 2.0
                # TODO: Fix time. Have to access waypoint marker menus, etc...
                # apply initial waypoint settings
                waypoint.joint_state.velocities = [0]*len(self.hebi_mapping)
                waypoint.joint_state.accelerations = [0]*len(self.hebi_mapping)
                waypoint.time = 0
            self.waypoints.insert(ref_index, waypoint)
        else:
            if ref_index < len(self.waypoints) - 1:
                self.waypoints.insert(ref_index+1, waypoint)
            else:
                # undo previous initial waypoint settings
                self.waypoints[-1].joint_state.velocities = [NAN]*len(self.hebi_mapping)
                self.waypoints[-1].joint_state.accelerations = [NAN]*len(self.hebi_mapping)
                # apply final waypoint settings
                waypoint.joint_state.velocities = [0]*len(self.hebi_mapping)
                waypoint.joint_state.accelerations = [0]*len(self.hebi_mapping)
                self.waypoints.append(waypoint)
            wp_passed_cnt += 1
        self.waypoints_cnt += 1
        self.int_markers_man.add_waypoint_marker(waypoint, description=str(wp_passed_cnt+1))
        self._prev_breakpoint = False

    def restart(self):
        self.waypoints = []
        self.waypoints_cnt = 0
        self.breakpoints_cnt = 0
        self._prev_breakpoint = True
        self.release_position()
        self.int_markers_man.clear_waypoint_markers()

    def hold_position(self):
        self._hold_joint_states = self._get_joint_angles()
        self._hold_position = True

    def release_position(self):
        self._hold_position = False

    def exit(self):
        self.release_position()

    def _get_joint_angles(self):
        return [self.current_jt_pos[motor] for motor in self.hebi_mapping]

    def _get_joint_velocities(self):
        return [self.current_jt_vel[motor] for motor in self.hebi_mapping]

    def _get_joint_efforts(self):
        return [self.current_jt_eff[motor] for motor in self.hebi_mapping]

    def _feedback_cb(self, msg):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.hebi_mapping:
                print("WARNING: arm_callback - unrecognized name!!!")
            else:
                self.current_jt_pos[name] = pos
                self.current_jt_vel[name] = vel
                self.current_jt_eff[name] = eff
                if not rospy.is_shutdown() and self._joint_state_pub is not None:  # Publish JointState() for RViz
                    jointstate = JointState()
                    jointstate.header.stamp = rospy.Time.now()
                    jointstate.name = self._active_joints
                    jointstate.position = self._get_joint_angles()
                    jointstate.velocity = [0.0] * len(jointstate.name)
                    jointstate.effort = [0.0] * len(jointstate.name)
                    self._joint_state_pub.publish(jointstate)

        # TODO: Make this less hacky
        if not rospy.is_shutdown() and self._joint_state_pub is not None:
            if not self._executing_trajectory:
                joint_grav_torques = self.kdl_kin.get_joint_torques_from_gravity(self._get_joint_angles(), grav=[0,0,-9.81])  # TODO: FIXME: Hardcoded
                jointstate = JointState()
                jointstate.header.stamp = rospy.Time.now()
                jointstate.name = self.hebi_mapping
                if self._hold_position:
                    # jointstate.position = self.waypoints[-1].positions  # jerky
                    jointstate.position = self._hold_joint_states
                else:
                    jointstate.position = []
                jointstate.velocity = []
                jointstate.effort = joint_grav_torques
                self.cmd_pub.publish(jointstate)
