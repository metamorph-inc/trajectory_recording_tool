#! /usr/bin/env python
"""
# Name: smach_state_classes.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 02/12/2018
# Edit Date: 02/12/2018
#
# Description:
# SMACH State Class to
# - load trajectory.json artifact produced by the trajectory_recording_tool
# - execute the trajectory expressed in the *.json file
#
# Provides a generic ExecuteJointTrajectoryFromFile SMACH state
# for composition of state machine behaviors using the trajectory_recording_tool.
"""

import os
import copy

import rospy
from rospkg import RosPack
from smach import State
from sensor_msgs.msg import JointState
from hebiros.msg import TrajectoryGoal, WaypointMsg

from json_load_byteified import json_load_byteified


class ExecuteJointTrajectoryFromFile(State):
    """SMACH state: Executes a joint trajectory

    Attributes:
        ros_package                     (str): ROS package containing trajectory file (ROS package/src/trajectories/)
        filename                        (str): trajectory filename (e.g. pick_up_cup_0.json)
        hebi_mapping                    (list of str): [Fam1/Name1, Fam2/Name2, ... FamN/NameN]
        trajectory_action_client        (obj): action client to send TrajectoryGoals to hebiros_node
        set_cmd_lifetime_service_client (obj): service client to send command lifetimes to hebiros_node
        hebi_fbk_topic                  (str): feedback topic from HEBI group
        hebi_cmd_topic                  (str): command topic to HEBI group
        setup_time                      (float): time [s] to move from current position to trajectory start position
    """
    def __init__(self, ros_package, filename, hebi_mapping, trajectory_action_client, set_cmd_lifetime_service_client,
                 hebi_fbk_topic, hebi_cmd_topic, setup_time=1.0):
        State.__init__(self, outcomes=['exit', 'failure', 'success'], output_keys=['final_joint_positions'])

        self._ros_package = ros_package
        self._filename = filename
        self._hebi_mapping = hebi_mapping
        self._trajectory_action_client = trajectory_action_client
        self._set_cmd_lifetime_service_client = set_cmd_lifetime_service_client
        self.fbk_sub = rospy.Subscriber(hebi_fbk_topic, JointState, self._feedback_cb)
        self.cmd_pub = rospy.Publisher(hebi_cmd_topic, JointState, queue_size=1)
        self._setup_time = setup_time

        self._active = False
        self._current_jt_pos = {}
        self._current_jt_vel = {}
        self._current_jt_eff = {}

        self._rate = rospy.Rate(200)

        # Load trajectory file
        trajectory_json = self._load_trajectory_from_file(self._ros_package, self._filename)
        # Parse trajectory file
        self._base_trajectory_goal = self._get_trajectory_goal_from_json_data(trajectory_json)

    def execute(self, userdata):
        self._active = True

        if self._setup_time != 0.0:
            # Add current joint data to front of base trajectory_goal
            trajectory_goal = self._insert_current_joint_data(copy.deepcopy(self._base_trajectory_goal), self._setup_time)
        else:
            trajectory_goal = self._base_trajectory_goal
        # Send goal to action server
        self._trajectory_action_client.send_goal(trajectory_goal)
        # Wait for server to finish performing the action
        self._trajectory_action_client.wait_for_result()
        # TODO: Feedback while action is executing?
        #       http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
        self._trajectory_action_client.get_result()  # TODO: return may be useful
        # Clean up and exit
        final_joint_positions = trajectory_goal.waypoints[-1].positions
        self._on_exit(userdata, final_joint_positions)
        return 'success'

    @staticmethod
    def _load_trajectory_from_file(ros_package, filename):
        """Loads trajectory from *.json file. Converts unicode strings to Python str for ROS.

        Args:
            ros_package (str): Name of ros_package where trajectory file is located
            filename (str): Name of *.json trajectory file
        Returns:
            obj: Python object from customized json.load(...) with an custom object_hook function
        """
        rospack = RosPack()
        package_path = rospack.get_path(ros_package)
        file_path = os.path.join(package_path, "trajectories", filename)
        with open(file_path, 'r') as loadfile:
            return json_load_byteified(loadfile)

    @staticmethod
    def _get_trajectory_goal_from_json_data(trajectory_json):
        """Converts Python object into hebiros TrajectoryGoal()

        Args:
            trajectory_json (obj): Python object from customized json.load(...) with an custom object_hook function
        Returns:
            obj: TrajectoryGoal() with current joint state appended to front
        """
        # Get data from json data structure
        waypoints = []
        for i in range(len(trajectory_json["waypoints"])):
            waypoints.append(trajectory_json["waypoints"][str(i)])
        times = trajectory_json["times"]
        # Create ROS msg
        trajectory_goal = TrajectoryGoal()
        for waypoint in waypoints:
            waypoint_msg = WaypointMsg()
            waypoint_msg.names = waypoint["names"]
            waypoint_msg.positions = waypoint["positions"]
            waypoint_msg.velocities = waypoint["velocities"]
            waypoint_msg.accelerations = waypoint["accelerations"]
            trajectory_goal.waypoints.append(waypoint_msg)
        trajectory_goal.times = times
        return trajectory_goal

    def _insert_current_joint_data(self, trajectory_goal, setup_time):
        """Inserts current joint data to front of trajectory goal

        Args:
            trajectory_goal (obj): Base TrajectoryGoal()
            setup_time (float): Time [s] to reach initial trajectory position from current joint state
        Returns:
            obj: TrajectoryGoal() with current joint state appended to front
        """
        # Wait for joint feedback before parsing trajectory file
        while not rospy.is_shutdown() and len(self._current_jt_eff) != len(self._hebi_mapping):
            self._rate.sleep()
        if rospy.is_shutdown():
            return 'exit'
        # Insert current joint values to front of trajectory.waypoints
        waypoint_msg = WaypointMsg()
        waypoint_msg.names = trajectory_goal.waypoints[0].names
        waypoint_msg.positions = [self._current_jt_pos[name] for name in self._hebi_mapping]
        waypoint_msg.velocities = [self._current_jt_vel[name] for name in self._hebi_mapping]
        waypoint_msg.accelerations = [self._current_jt_eff[name] for name in self._hebi_mapping]
        trajectory_goal.waypoints.insert(0, waypoint_msg)
        # Increment trajectory.times by setup_time then insert 0 to front of trajectory.times
        trajectory_goal.times = [setup_time + time for time in trajectory_goal.times]
        trajectory_goal.times.insert(0, 0.0)
        print(trajectory_goal.times)
        print(trajectory_goal.waypoints)
        return trajectory_goal

    def _feedback_cb(self, msg):
        if self._active:
            for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name not in self._hebi_mapping:
                    print("WARNING: arm_callback - unrecognized name!!!")
                else:
                    self._current_jt_pos[name] = pos
                    self._current_jt_vel[name] = vel
                    self._current_jt_eff[name] = eff

    @staticmethod
    def _update_userdata(userdata, final_joint_positions):
        """Updates userdata dictionary that gets passed to next State (Container) instance

        Args:
            final_joint_positions (list of float): Final joint positions from TrajectoryGoal
        """
        userdata.final_joint_positions = final_joint_positions

    def _on_exit(self, userdata, final_joint_positions):
        """Updates userdata dictionary, sets self._active_flag = False,

        Args:
            final_joint_positions (list of float): Final joint positions from TrajectoryGoal
        """
        self._update_userdata(userdata, final_joint_positions)
        self._active = False
        self._current_jt_pos = {}
        self._current_jt_vel = {}
        self._current_jt_eff = {}


# TODO: This should really be moved to a generic States package

from std_msgs.msg import String


class BreakOnMsg(State):
    """SMACH state: Returns 'true' when a message is received. Returns 'false' otherwise

    Attributes:
        listen_topic    (str): listen topic
    """
    def __init__(self, listen_topic):
        State.__init__(self, outcomes=['exit', 'true', 'false'])
        # ROS stuff
        self._listen_sub = rospy.Subscriber(listen_topic, String, self._listen_callback)
        self._msg_from_master = None

    def execute(self, userdata):
        if self._msg_from_master is None:
            return 'false'
        else:
            self._msg_from_master = None
            return 'true'

    def _listen_callback(self, msg):
        self._msg_from_master = msg.data
        print("received msg")
