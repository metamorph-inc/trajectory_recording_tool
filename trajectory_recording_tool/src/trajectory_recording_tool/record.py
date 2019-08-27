#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# Name: record.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Email: jcoombe@metamorphsoftware.com
# Create Date: 02/07/2018
# Edit Date: 02/07/2018
#
# Description:
# Records a series of trajectory waypoints with user-specified times/velocities.
# Saves the waypoints to a [ROS_PACKAGE]/recorded_trajectories/[NAME].csv
# where [ROS_PACKAGE] and [NAME] are specified by the user.
#
# Generated trajectory *.csv files can be loaded into a generic
# execute_jt_trajectory SMACH state instance to compose state machine behaviors.
"""
from __future__ import print_function

import sys
import argparse

import rospy

from trajectory_recorder_class import TrajectoryRecorder


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-hebi_group', type=str, required=True, help="hebi_group_name")
    parser.add_argument('-modules', type=str, nargs='+', required=True,
                        help="Module1Family/Module1Name Module2Family/Module2Name ... ModuleNFamily/ModuleNName")
    return parser.parse_args(args)


def main():

    # Parse args
    parser = parse_args(rospy.myargv(sys.argv[1:]))
    rospy.loginfo("parser.hebi_group", parser.hebi_group)
    rospy.loginfo("parser.modules", parser.modules)

    # ROS setup
    rospy.init_node('trajectory_recording_tool')
    rate = rospy.Rate(200)

    # Create TrajectoryRecorder instance
    hebi_group_name = parser.hebi_group
    hebi_families = [module.split('/')[0] for module in parser.modules]
    hebi_names = [module.split('/')[1] for module in parser.modules]
    recorder = TrajectoryRecorder(hebi_group_name, hebi_families, hebi_names)

    # Main Menu loop
    finished = False
    while not rospy.is_shutdown() and not finished:
        print("\nMain Menu - Commands:")
        print("  [a] or [add]      - add Waypoints to Trajectory.")
        print("  [b] or [break]    - add Breakpoint to Trajectory.")
        print("  [p] or [print]    - print Trajectory.")
        print("  [e] or [execute]  - execute Trajectory.")
        print("  [s] or [save]     - save Trajectory to a file.")
        print("  [r] or [reset]    - start a new Trajectory [WARNING: This will delete previous points].")
        print("  [exit]            - exit tool.")
        user_input = recorder.raw_input_ros_safe("  ").lower()
        if user_input == "a" or user_input == "add":
            # Add Waypoints to Trajectory
            recorder.release_position()
            back = False
            while not rospy.is_shutdown() and not back:
                print("\nAdd Waypoint - Commands:")
                print("  [Return] - record the current position as a Waypoint.")
                print("  [m] - record a continuous movement as a series of Waypoints.")
                print("  [Any other key] - return to Main Menu.")
                user_input = recorder.raw_input_ros_safe("  ")
                if user_input == " " or user_input == "":
                    recorder.set_waypoint()
                    recorder.print_waypoints()
                elif user_input == "m":
                    recorder.record_movement()  # FIXME: Need to implement
                else:
                    back = True

        elif user_input == "b" or user_input == "break":
            # Add Breakpoint
            print("\nAdding Breakpoint...")
            recorder.append_breakpoint()
            recorder.print_waypoints()
            print("Done.")

        elif user_input == "p" or user_input == "print":
            # Print Trajectory
            print("\nPrinting Trajectory...")
            recorder.print_waypoints()
            print("Done.")

        elif user_input == "e" or user_input == "execute":
            # Execute Trajectory
            print("\nExecuting Trajectory...")
            recorder.release_position()
            recorder.execute_trajectories()
            print("Done.")

        elif user_input == "s" or user_input == "save":
            # Save Trajectory
            print("\nSaving Trajectory...")
            recorder.save_trajectory_to_file()
            print("Done.")

        elif user_input == "r" or user_input == "reset":
            # Restart
            print("\nResetting... Deleting old points...")
            recorder.restart()
            print("\nDone.")

        elif user_input == "exit":
            # exit tool
            recorder.exit()
            print("\nGoodbye.")
            finished = True

        else:
            # prompt user for valid input
            print("\nInput: \"{}\" not recognized. Please try again.".format(user_input))


if __name__ == '__main__':
    main()

# TODO: Add unittests
