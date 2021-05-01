#!/usr/bin/env python
import os
import sys
import copy
import datetime
import csv
import numpy as np
import rospy
import rosparam
import moveit_commander
import geometry_msgs
import visualization_msgs
import std_msgs
import moveit_msgs
from tf.transformations import (
    quaternion_about_axis,
    quaternion_multiply,
    quaternion_matrix,
)
from moveit_commander.conversions import pose_to_list

# Ref: move_group_python_interface_tutorial.py
# https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

"""
tolerance
xyz: 0.01 cm
xyzw: 0.01 (about 1.2 deg)
"""


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True


class MoveGroupPythonInteface(object):

    """MoveGroupPythonInteface"""

    def __init__(self, _group_name, ee_name=None):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        # group_name = rosparam.get_param("%s/group_name" % rospy.get_name())
        group_name = _group_name

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander(group_name)
        if ee_name is not None:
            move_group.set_end_effector_link(ee_name)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # Get the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # Get a list of all the groups in the robot:
        group_names = robot.get_group_names()

        print "============ INFO ============"
        print "Available Planning Groups: ", robot.get_group_names()
        rospy.logwarn("group_name: %s" % group_name)
        print "Planning frame: %s" % planning_frame
        print "End effector link: %s" % eef_link
        print "Robot joint space:"
        print robot.get_current_state().joint_state
        print "=============================="
        print ""

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.joint_names = []
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    # def force_set_joint_values(self, joint_values):
    #     idx = 0
    #     for joint_value in joint_values:
    #         name = self.joint_names[idx]
    #         self.robot.get_joint(name).move(joint_value)
    #         idx += 1
    #     self.move_group.stop()

    #     current_joints = self.move_group.get_current_joint_values()
    #     return all_close(joint_values, current_joints, 0.01)

    def go_to_joint_state(self, joint_goal):
        # The Panda's zero configuration is at a
        # `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`

        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # tip
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position, quat):
        """
        position: geometry_msgs/Point
        quat: geometry_msgs/Quaternion
        """
        # end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = quat
        pose_goal.position = position

        self.move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        # True, False
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

        # tip
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.02)

    def get_current_joints(self):
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        return self.move_group.get_current_pose().pose

    def check_current_pose(self, pose_goal, tolerance=0.01):
        current_pose = self.move_group.get_current_pose().pose
        rospy.loginfo(">> pose_goal: %s\n", pose_goal)
        rospy.loginfo(">> current_pose: %s\n", current_pose)
        return all_close(pose_goal, current_pose, tolerance)

    def get_manipulability(self, joint_values):
        jacobian_matrix = self.move_group.get_jacobian_matrix(joint_values)
        # jacobian_matrix.shape is '6 x joints'

        A_mat = np.matmul(jacobian_matrix, np.transpose(jacobian_matrix))
        # A_mat.shape is '6 x 6'
        eigen_values, eigen_vectors = np.linalg.eig(A_mat)

        # [ Manipulability ]
        # {Method 1} >= 1 (Smaller is better)
        # The ratio of longest and shortest axes
        # condition_number = np.max(eigen_values) / np.min(eigen_values)
        # ratio = np.sqrt(condition_number)

        # {Method 2} >= 1 (Smaller is better)
        # Use condition number of A as manipulability value
        # condition_number = np.max(eigen_values) / np.min(eigen_values)

        # {Method 3}
        # proportional to volume of ellipsoid
        # mu = np.sqrt(np.prod(eigen_values))

        # {Method 4}: MoveIt CPP Version <= 1, >=0 (Bigger is better)
        # **Ref**: http://docs.ros.org/en/jade/api/moveit_core/html/classkinematics__metrics_1_1KinematicsMetrics.html#a8bc0ff4bbb402031460232dfe9a1f18d
        # Get the manipulability = sigma_min/sigma_max where sigma_min and sigma_max are
        # the smallest and largest singular values of the Jacobian matrix J.
        # (The singular values in S are square roots of eigenvalues from J*JT or JT*J.)
        mu = np.sqrt(np.min(eigen_values) / np.max(eigen_values))

        return mu

    def plan_cartesian_path(self, pose):
        waypoints = [pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector

        box_name = self.box_name
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # For the Panda robot, we set ``grasping_group = 'hand'``.
        # If you are using a different robot, you should change this value
        # to the name of your end effector group name.
        group_names = self.group_names
        grasping_group = "hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # **Note:** The object must be detached before we can remove it from the world
        self.scene.remove_world_object(self.box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


class WorkspaceSpreader:
    def __init__(self, initial_pose):
        position = self.position_tuple_from_pose(initial_pose)
        self.open_list = set()
        self.closed = set()
        self.open_list.add(position)

        self.dx = 0.05
        self.dy = 0.05
        self.dz = 0.05

    def is_available(self):
        return len(self.open_list) > 0

    def next(self):
        position = self.open_list.pop()
        self.closed.add(position)
        return position

    def mark_in_workspace(self, pose):
        position = self.position_tuple_from_pose(pose)
        self.closed.add(position)
        for p in self.around(position):
            if p not in self.closed:
                # If you try to add an already existing element to a Set, it won't do anything.
                self.open_list.add(p)

    def around(self, position):
        for dx in [self.dx, 0.0, -self.dx]:
            for dy in [self.dy, 0.0, -self.dy]:
                for dz in [self.dz, 0.0, -self.dz]:
                    new_p = (position[0] + dx, position[1] + dy, position[2] + dz)
                    if new_p != position:
                        if new_p[2] <= 0.8 and new_p[2] >= 0.7:
                            yield new_p

    @staticmethod
    def position_tuple_from_pose(pose):
        return (pose.position.x, pose.position.y, pose.position.z)


"""
  <!-- end-effector tf  -->
  <node pkg="tf"
        type="static_transform_publisher"
        name="left_eef_tf"
        args="0 0 0.109 0 0 1 0 LHand_base left_end_effect_point 100" />
  <node pkg="tf"
        type="static_transform_publisher"
        name="right_eef_tf"
        args="0 0 0.109 0 0 0 1 RHand_base right_end_effect_point 100" />
"""


class Recorder:
    def __init__(self):
        rospy.init_node("manipulability_recorder", anonymous=True)
        print ""
        print "----------------------------------------------------------"
        print " Manipulability Recorder"
        print "----------------------------------------------------------"
        print ""
        # _frame_id = "panda_link0"
        # _group_name = "panda_arm"
        # _moveit_eef = "panda_hand"
        _init_ws_pose = None

        _frame_id = "base_footprint"
        _group_name = "right_arm"
        _moveit_eef = "RHand_base"  # default
        _init_ws_pose = geometry_msgs.msg.Pose()
        _init_ws_pose.position.x = 0.2 + 0.109
        _init_ws_pose.position.y = -0.3
        _init_ws_pose.position.z = 0.75
        # The interest_eef pose relative to moveit_eef
        self.interest_eef_offset = geometry_msgs.msg.Pose()
        self.interest_eef_offset.position.z = 0.109
        self.interest_eef_offset.orientation.w = 1.0

        self.moveit = MoveGroupPythonInteface(_group_name, ee_name=_moveit_eef)
        if _init_ws_pose is None:
            _init_ws_pose = self.moveit.get_current_pose()
        self.ws = WorkspaceSpreader(_init_ws_pose)

        self.save_joint_name_only_once = True
        self.joint_names = []

        # Marker
        self.pub = rospy.Publisher(
            "rviz_visual_tools", visualization_msgs.msg.MarkerArray, queue_size=1
        )

        # Markers
        p = geometry_msgs.msg.Point

        def make_maker():
            m = visualization_msgs.msg.Marker()
            m.header.frame_id = _frame_id
            m.header.stamp = rospy.Time()
            m.action = visualization_msgs.msg.Marker.ADD
            return m

        # Target
        self.tg_marker_x = make_maker()
        self.tg_marker_x.type = visualization_msgs.msg.Marker.ARROW
        self.tg_marker_x.lifetime.secs = 30
        self.tg_marker_x.id = 200
        self.tg_marker_x.scale.x = 0.15  # arrow length
        self.tg_marker_x.scale.y = 0.015  # arrow width
        self.tg_marker_x.scale.z = 0.03  # arrow height
        self.tg_marker_x.color.a = 0.6
        self.tg_marker_x.color.r = 0.93
        self.tg_marker_x.color.g = 0.18
        self.tg_marker_x.color.b = 0.93
        self.tg_marker_z = make_maker()
        self.tg_marker_z.type = visualization_msgs.msg.Marker.ARROW
        self.tg_marker_z.lifetime.secs = 30
        self.tg_marker_z.id = 201
        self.tg_marker_z.scale.x = 0.15  # arrow length
        self.tg_marker_z.scale.y = 0.015  # arrow width
        self.tg_marker_z.scale.z = 0.03  # arrow height
        self.tg_marker_z.color.a = 0.6
        self.tg_marker_z.color.r = 0.0
        self.tg_marker_z.color.g = 0.0
        self.tg_marker_z.color.b = 1.0

        # Manipulability
        self.m_marker = make_maker()
        self.m_marker.type = visualization_msgs.msg.Marker.ARROW
        self.m_marker.scale.x = 0.03  # arrow length
        self.m_marker.scale.y = 0.004  # arrow width
        self.m_marker.scale.z = 0.008  # arrow height
        # ID_manip: 300 ~

        # Out of workspace
        self.o_marker = copy.deepcopy(self.m_marker)
        self.o_marker.color.a = 0.2
        self.o_marker.color.r = 0.4
        self.o_marker.color.g = 0.4
        self.o_marker.color.b = 0.4
        self.markers = visualization_msgs.msg.MarkerArray()

    def manip_color(self, manip):
        m = min(1.0, manip * 5.0)
        return std_msgs.msg.ColorRGBA(r=1.0 - m, g=m, b=0.0, a=1.0)

    @staticmethod
    def quat_msg_rotation(quat, rad, axis):
        q = (
            quat
            if isinstance(quat, np.ndarray)
            else np.array([quat.x, quat.y, quat.z, quat.w])
        )
        quat_array = quaternion_multiply(q, quaternion_about_axis(rad, axis))
        new_quat = geometry_msgs.msg.Quaternion()
        new_quat.x = quat_array[0]
        new_quat.y = quat_array[1]
        new_quat.z = quat_array[2]
        new_quat.w = quat_array[3]
        return new_quat

    @staticmethod
    def get_last_joint_from_plan(plan):
        traj = plan.joint_trajectory.points
        joints = traj[-1].positions if len(traj) > 0 else []
        return joints

    def write_titles(self, writer):
        titles = [
            "TCP x",
            "TCP y",
            "TCP z",
            "eef x",
            "eef y",
            "eef z",
            "Cr(rzyz)",
            "Pitch(rzyz)",
            "Roll(rzyz)",
        ]
        titles += ["Manipulability"]
        titles += self.joint_names
        writer.writerow(titles)

    def try_target(self, eef_pose, tcp_pose, count):
        manipulability = None
        joint_values = None
        try:
            self.tg_marker_x.pose = copy.deepcopy(eef_pose)
            self.tg_marker_z.pose = copy.deepcopy(eef_pose)
            self.tg_marker_z.pose.orientation = self.quat_msg_rotation(
                eef_pose.orientation, np.radians(-90), (0, 1, 0)
            )
            markers = visualization_msgs.msg.MarkerArray()
            markers.markers.append(self.tg_marker_x)
            markers.markers.append(self.tg_marker_z)
            self.pub.publish(markers)

            plan, fraction = self.moveit.plan_cartesian_path(copy.deepcopy(eef_pose))
            # ######################
            # self.moveit.execute_plan(plan)
            # rospy.sleep(2)
            # success = self.moveit.check_current_pose(eef_pose, tolerance=0.02)
            # if success:
            #     if self.save_joint_name_only_once:
            #         self.joint_names = [
            #             name for name in plan.joint_trajectory.joint_names
            #         ]
            #     joints = self.get_last_joint_from_plan(plan)
            #     joint_values = list(joints)
            #     manipulability = self.moveit.get_manipulability(joint_values)
            # ######################

            # ######################
            success = self.moveit.go_to_pose_goal(
                copy.deepcopy(eef_pose.position), copy.deepcopy(eef_pose.orientation)
            )
            if success:
                if self.save_joint_name_only_once:
                    self.joint_names = [
                        name for name in plan.joint_trajectory.joint_names
                    ]
                    rospy.logwarn(self.joint_names)
                joint_values = self.moveit.get_current_joints()
                manipulability = self.moveit.get_manipulability(joint_values)
            # ######################

            # ######################
            # joints = self.get_last_joint_from_plan(plan)
            # if joints:
            #     if self.save_joint_name_only_once:
            #         self.joint_names = [
            #             name for name in plan.joint_trajectory.joint_names
            #         ]
            #     rospy.logwarn("joint True:")
            #     rospy.logwarn(joints)
            #     success = True

            #     state_msg = self.moveit.move_group.get_current_state()
            #     print "==============="
            #     print "self.moveit.move_group.get_current_state()"
            #     print state_msg
            #     print "==============="
            #     print "joints"
            #     print self.joint_names
            #     print joints
            #     old_position = list(state_msg.joint_state.position)
            #     for i in range(len(self.joint_names)):
            #         jname = self.joint_names[i]
            #         jvalue = joints[i]
            #         msg_idx = state_msg.joint_state.name.index(jname)
            #         old_position[msg_idx] = jvalue
            #     state_msg.joint_state.position = old_position
            #     self.moveit.move_group.set_start_state(state_msg)
            #     self.moveit.move_group.set_start_state_to_current_state()

            #     print "==============="
            #     print "self.moveit.move_group.get_current_state()"
            #     print self.moveit.move_group.get_current_state()
            #     print "==============="
            #     # state_msg = moveit_msgs.msg.RobotState()
            # else:
            #     rospy.logwarn("joint False:")
            #     rospy.logwarn(joints)
            #     success = False

            if success:
                print "[count: %4d] %.5f" % (count, manipulability)
            else:
                print "[count: %4d] out of workspace" % (count)

            this_marker = copy.deepcopy(self.m_marker if success else self.o_marker)
            this_marker.id = count + 300
            this_marker.pose.position = tcp_pose.position
            this_marker.pose.orientation = self.quat_msg_rotation(
                eef_pose.orientation, np.radians(-90), (0, 1, 0)
            )
            if success:
                this_marker.color = self.manip_color(manipulability)

            self.markers.markers.append(this_marker)
            # print len(self.markers.markers)
            # print self.markers.markers
            # raw_input()

            self.pub.publish(self.markers)
            if count > 127:
                self.markers.markers.pop(0)
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            pass
        return manipulability, joint_values

    def main(self):
        approach_set = [
            (
                Cr,
                self.quat_msg_rotation(
                    quaternion_about_axis(np.radians(Cr), (0, 0, 1)),
                    np.radians(90),
                    (0, 1, 0),
                ),
            )
            for Cr in range(-90, 91, 15)
        ]

        ymd_hms = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_name = "socialrobot_manip_%s.csv" % ymd_hms
        directory = "/home/francisco/catkin_ws/src/Robot-Manipulability/log/"
        with open(os.path.join(directory, file_name), "w") as csvfile:
            writer = csv.writer(csvfile)

            count = np.radians(-90)0
            while self.ws.is_available():
                position = self.ws.next()

                p = geometry_msgs.msg.Pose()
                p.position.x = position[0]
                p.position.y = position[1]
                p.position.z = position[2]

                once_possible = False
                for Cr, quat in approach_set:
                    # print "============ Next?"
                    # raw_input()
                    p.orientation = quat
                    # offset
                    corrected_p = self.eef_correction(p)
                    manipulability, joint_values = self.try_target(
                        corrected_p, p, count
                    )
                    # rospy.logwarn("joint_values: %s" % joint_values)
                    if manipulability is not None:
                        if self.save_joint_name_only_once:
                            self.save_joint_name_only_once = False
                            self.write_titles(writer)
                        once_possible = True
                        # save
                        writer.writerow(
                            [
                                p.position.x,
                                p.position.y,
                                p.position.z,
                                corrected_p.position.x,
                                corrected_p.position.y,
                                corrected_p.position.z,
                                Cr,
                                90.0,
                                -90.0,
                                manipulability,
                                joint_values[0],
                                joint_values[1],
                                joint_values[2],
                                joint_values[3],
                                joint_values[4],
                                joint_values[5],
                                joint_values[6],
                                joint_values[7],
                            ]
                        )
                    count += 1
                if once_possible:
                    self.ws.mark_in_workspace(p)
                rospy.sleep(0.01)

    def eef_correction(self, pose):
        corrected_p = copy.deepcopy(pose)
        # Move pose.position by -interest_eef_offset.position based on pose.orientation
        quat = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        mat = quaternion_matrix(quat)

        xyz_offset = np.array(
            [
                -self.interest_eef_offset.position.x,
                -self.interest_eef_offset.position.y,
                -self.interest_eef_offset.position.z,
                1.0,
            ]
        )
        xyz_offset = np.matmul(mat, xyz_offset)
        corrected_p.position.x += xyz_offset[0]
        corrected_p.position.y += xyz_offset[1]
        corrected_p.position.z += xyz_offset[2]

        # Rotate pose.orientation by conjugate(interest_eef_offset.orientation)
        return corrected_p


if __name__ == "__main__":
    r = Recorder()
    print "============ Press `Enter` to start recording ..."
    raw_input()
    r.main()
