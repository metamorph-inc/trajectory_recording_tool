"""
# Name: rviz_interactive_markers.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 03/14/2018
# Edit Date: 03/20/2018
#
# Description:
#     Interactive markers in RViz for trajectory_recording_tool
# References:
#     https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/basic_controls.py
"""
# TODO: My scheme for tracking marker/waypoint objects could probably be improved if we want to develop this further.

import math as m
import copy

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerInit, InteractiveMarkerControl, \
    InteractiveMarkerPose, InteractiveMarkerUpdate, InteractiveMarkerFeedback, Marker


def make_box_marker(msg):
    # TODO: Possibly abstract to support other shapes/settings
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.9
    return marker


def make_box_control_marker(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_box_marker(msg))
    msg.controls.append(control)
    return control


class InteractiveMarkerManager(object):
    def __init__(self, parent_instance_handle, server_name, ik_solver):
        self.parent_instance_handle = parent_instance_handle
        self.server = InteractiveMarkerServer(server_name)
        self.ik_solver = ik_solver
        self.ik_position_xyz_bounds = [0.01, 0.01, 0.01]
        self.ik_position_rpy_bounds = [31416.0, 31416.0, 31416.0]  # NOTE: This implements position-only IK

        self._menu_handlers = {}
        self._menu_cmds = {}

        self.menu_handler = MenuHandler()
        del_sub_menu_handle = self.menu_handler.insert("Delete Waypoint")
        self._menu_cmds['del_wp'] = self.menu_handler.insert("Yes", parent=del_sub_menu_handle, callback=self._process_feedback)
        self.menu_handler.insert("No", parent=del_sub_menu_handle, callback=self._process_feedback)
        ins_sub_menu_handle = self.menu_handler.insert("Insert Waypoint")
        self._menu_cmds['ins_wp_before'] = self.menu_handler.insert("Before", parent=ins_sub_menu_handle, callback=self._process_feedback)
        self._menu_cmds['ins_wp_after'] = self.menu_handler.insert("After", parent=ins_sub_menu_handle, callback=self._process_feedback)
        self.menu_handler.insert("Cancel", parent=ins_sub_menu_handle, callback=self._process_feedback)

        self._int_marker_name_list = []
        self.markers_created_cnt = 0
        self.marker_cnt = 0
        self.name_to_marker_dict = {}
        self.waypoint_to_marker_dict = {}
        self.name_to_waypoint_dict = {}

    def _process_feedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.logdebug(s + ": button click" + mp + ".")
            pass
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.logdebug(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
            self._process_menu_select(feedback)
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:  # TODO: I really want to use the other type of feedback
            rospy.logdebug(s + ": pose changed")
            self._update_waypoint_position(feedback.marker_name, feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.logdebug(s + ": mouse down" + mp + ".")
            pass
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.logdebug(s + ": mouse up" + mp + ".")
            pass
        self.server.applyChanges()

    def _process_menu_select(self, feedback):
        menu_entry_id = feedback.menu_entry_id
        if menu_entry_id == self._menu_cmds['del_wp']:
            self._delete_waypoint(feedback)
        elif menu_entry_id == self._menu_cmds['ins_wp_before']:
            self._insert_waypoint(feedback, before=True)
        elif menu_entry_id == self._menu_cmds['ins_wp_after']:
            self._insert_waypoint(feedback, before=False)
        elif menu_entry_id in self._menu_cmds[feedback.marker_name]['menu_time_cmds']:
            self._process_menu_time_cmds(feedback)
        else:
            pass

    def _delete_waypoint(self, feedback):
        marker = self.name_to_marker_dict[feedback.marker_name]
        waypoint = self.name_to_waypoint_dict[marker.name]
        self.parent_instance_handle.delete_waypoint(waypoint)
        # Update dictionaries
        del self.name_to_marker_dict[marker.name]
        del self.waypoint_to_marker_dict[waypoint]
        del self.name_to_waypoint_dict[marker.name]
        del self._menu_handlers[marker.name]
        del self._menu_cmds[marker.name]
        # Erase marker from server
        self.server.erase(feedback.marker_name)
        self.server.applyChanges()
        self.marker_cnt -=1
        self._int_marker_name_list.remove(marker.name)
        self._update_wp_marker_descriptions()
        self.server.applyChanges()

    def _insert_waypoint(self, feedback, before=True):
        ref_marker = self.name_to_marker_dict[feedback.marker_name]
        ref_waypoint = self.name_to_waypoint_dict[ref_marker.name]
        self.parent_instance_handle.insert_waypoint(ref_waypoint, before=before)
        self._update_wp_marker_descriptions()
        self.server.applyChanges()

    def _process_menu_time_cmds(self, feedback):
        menu_entry_handle = feedback.menu_entry_id
        menu_handler = self._menu_handlers[feedback.marker_name]
        check_state = menu_handler.getCheckState(menu_entry_handle)

        if check_state == MenuHandler.UNCHECKED:
            menu_handler.setCheckState(menu_entry_handle, MenuHandler.CHECKED)
            # Uncheck all other menu entries
            for menu_entry_id in self._menu_cmds[feedback.marker_name]['menu_time_cmds']:
                if menu_entry_id != menu_entry_handle:
                    menu_handler.setCheckState(menu_entry_id, MenuHandler.UNCHECKED)
            # Update waypoints
            waypoint = self.name_to_waypoint_dict[feedback.marker_name]
            waypoint.time = self._menu_cmds[feedback.marker_name]['menu_time_cmds'][menu_entry_handle]

            # Apply marker changes
            menu_handler.reApply(self.server)
            self.server.applyChanges()

    def _update_waypoint_pose(self):
        # TODO maybe handle pose changes differently?
        pass

    def _update_waypoint_position(self, marker_name, marker_pose):
        waypoint = self.name_to_waypoint_dict[marker_name]
        eff_pos = marker_pose.position
        eff_orient = marker_pose.orientation
        target_jt_angles = self.ik_solver.get_ik(waypoint.joint_state.positions,
                                                 eff_pos.x, eff_pos.y, eff_pos.z,
                                                 eff_orient.w, eff_orient.x, eff_orient.y, eff_orient.z,
                                                 self.ik_position_xyz_bounds[0],
                                                 self.ik_position_xyz_bounds[1],
                                                 self.ik_position_xyz_bounds[2],
                                                 self.ik_position_rpy_bounds[0],
                                                 self.ik_position_rpy_bounds[1],
                                                 self.ik_position_rpy_bounds[2])
        if target_jt_angles is not None:
            waypoint.end_effector_pose = copy.deepcopy(marker_pose)
            waypoint.joint_state.positions = target_jt_angles
            self._change_wp_marker_color(marker_name, "white", 0.9)
        else:
            self._change_wp_marker_color(marker_name, "red", 0.9)

    def _change_wp_marker_color(self, wp_marker_name, color, opacity=1.0):
        assert 0.0 <= opacity <= 1.0
        # set color
        int_marker = self.server.get(wp_marker_name)
        marker = int_marker.controls[0].markers[0]
        replacement_int_marker = copy.deepcopy(int_marker)
        replacement_marker = replacement_int_marker.controls[0].markers[0]
        rgb = None
        if color == "white":
            rgb = (1.0, 1.0, 1.0)
        elif color == "red":
            rgb = (1.0, 0.0, 0.0)
        # TODO: Additional colors here
        changed = False
        if rgb is not None:
            if marker.color.r != rgb[0]:
                replacement_marker.color.r = rgb[0]
                changed = True
                print("change to ", rgb)
            if marker.color.g != rgb[1]:
                replacement_marker.color.g = rgb[1]
                changed = True
            if marker.color.b != rgb[2]:
                replacement_marker.color.b = rgb[2]
                changed = True
            if marker.color.a != opacity:
                replacement_marker.color.a = opacity
                changed = True
        # update wp marker
        if changed:
            self._replace_wp_marker(int_marker, replacement_int_marker)

    def _update_wp_marker_descriptions(self):
        for index, marker_name in enumerate(self._int_marker_name_list):
            marker = self.name_to_marker_dict[marker_name]
            marker_description = int(marker.description)
            if marker_description != index+1:
                marker.description = str(index+1)
                replacement_marker = copy.deepcopy(self.server.get(marker.name))
                replacement_marker.description = marker.description
                self._replace_wp_marker(marker, replacement_marker)

    def _replace_wp_marker(self, old_int_marker, replacement_int_marker):
        # Update dictionaries
        self.name_to_marker_dict[old_int_marker.name] = replacement_int_marker
        self.waypoint_to_marker_dict[self.name_to_waypoint_dict[old_int_marker.name]] = replacement_int_marker
        # Erase marker from server
        self.server.erase(old_int_marker.name)
        self.server.insert(replacement_int_marker, self._process_feedback)
        self.server.applyChanges()

    def add_waypoint_marker(self, waypoint, description=None):
        self.markers_created_cnt += 1
        self.marker_cnt += 1

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = waypoint.end_effector_pose.position
        int_marker.scale = 0.1
        int_marker.name = str(self.markers_created_cnt)
        if description is None:
            int_marker.description = int_marker.name
        else:
            int_marker.description = description
        self.name_to_marker_dict[int_marker.name] = int_marker
        self.waypoint_to_marker_dict[waypoint] = int_marker
        self.name_to_waypoint_dict[int_marker.name] = waypoint

        # insert a box  # TODO: may change geometry / eff mesh
        make_box_control_marker(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self._process_feedback)

        # add menus
        menu_handler = MenuHandler()
        del_sub_menu_handle = menu_handler.insert("Delete Waypoint")
        self._menu_cmds['del_wp'] = menu_handler.insert("Yes", parent=del_sub_menu_handle,
                                                        callback=self._process_feedback)
        menu_handler.insert("No", parent=del_sub_menu_handle, callback=self._process_feedback)
        ins_sub_menu_handle = menu_handler.insert("Insert Waypoint")
        self._menu_cmds['ins_wp_before'] = menu_handler.insert("Before", parent=ins_sub_menu_handle,
                                                               callback=self._process_feedback)
        self._menu_cmds['ins_wp_after'] = menu_handler.insert("After", parent=ins_sub_menu_handle,
                                                              callback=self._process_feedback)
        menu_handler.insert("Cancel", parent=ins_sub_menu_handle, callback=self._process_feedback)
        # Set time
        time_sub_menu_handle = menu_handler.insert("Set Waypoint Time")
        menu_time_cmds = {}
        time = self.name_to_waypoint_dict[int_marker.name].time
        time_list = [float(num) for num in range((int(m.ceil(time))))]
        time_list.append(time)
        time_list.extend([m.floor(time) + cnt + 1 for cnt in range(6)])
        self._menu_cmds[int_marker.name] = {"menu_time_cmds": menu_time_cmds}
        for t in time_list:
            time_opt_handle = menu_handler.insert(str(t), parent=time_sub_menu_handle, callback=self._process_feedback)
            menu_time_cmds[time_opt_handle] = t
            if t == time:
                menu_handler.setCheckState(time_opt_handle, MenuHandler.CHECKED)
            else:
                menu_handler.setCheckState(time_opt_handle, MenuHandler.UNCHECKED)
        menu_handler.apply(self.server, int_marker.name)
        self._menu_handlers[int_marker.name] = menu_handler
        self.server.applyChanges()

        if int(int_marker.description)-1 < len(self._int_marker_name_list):
            self._int_marker_name_list.insert(int(int_marker.description) - 1, int_marker.name)
        else:
            self._int_marker_name_list.append(int_marker.name)

    def clear_waypoint_markers(self):
        self.server.clear()
        self.server.applyChanges()

        self._menu_handlers = {}
        self._menu_cmds = {}

        self._int_marker_name_list = []
        self.markers_created_cnt = 0
        self.marker_cnt = 0

        self.name_to_marker_dict = {}
        self.waypoint_to_marker_dict = {}
        self.name_to_waypoint_dict = {}
