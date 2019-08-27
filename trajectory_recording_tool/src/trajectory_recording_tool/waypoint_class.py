"""
# Name: waypoint_class.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 03/16/2018
# Edit Date: 03/19/2018
#
# Description:
#     # TODO
"""

from hebiros.msg import WaypointMsg
from geometry_msgs.msg import Pose


class Waypoint(object):
    """
    Attributes

    """
    def __init__(self):
        self.joint_state = WaypointMsg()
        self.end_effector_pose = Pose()
        self.time = None
