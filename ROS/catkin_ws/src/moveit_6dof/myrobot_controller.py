#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from ROS_IGTL_Bridge.msg import igtlpoint  # Adjust the import statement to match your package structure

class PointMover:
    def __init__(self):
        self.points = []
        self.move_group = None

        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pose_goal', anonymous=True)

        # Initialize robot and move group commander
        robot = moveit_commander.RobotCommander()
        group_name = "main_planning_group"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Subscribe to the /IGTL_POINT_IN topic to receive points
        rospy.Subscriber('/IGTL_POINT_IN', igtlpoint, self.point_callback)

    def point_callback(self, msg):
        """Callback function to process the incoming point message."""
        # Convert point data from millimeters to meters
        point_in_meters = geometry_msgs.msg.Point()
        point_in_meters.x = msg.pointdata.x / 1000.0
        point_in_meters.y = msg.pointdata.y / 1000.0
        point_in_meters.z = msg.pointdata.z / 1000.0

        rospy.loginfo(f"Received point (meters): {point_in_meters.x}, {point_in_meters.y}, {point_in_meters.z}")

        # Store the point
        self.points.append(point_in_meters)

        # If two points have been received, move the robot
        if len(self.points) == 2:
            self.move_to_points()

    def move_to_points(self):
        """Move the robot to the received points sequentially using Cartesian path planning."""
        waypoints = []

        for point in self.points:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 1.0
            pose_goal.position = point
            waypoints.append(pose_goal)

        # Plan the Cartesian path connecting the waypoints
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Check if the Cartesian path planning was successful
        if fraction == 1.0:
            rospy.loginfo("Cartesian path planned successfully")
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logwarn("Cartesian path planning failed")

        # Stop any residual movement and clear targets
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        for point in self.points:
            rospy.loginfo(f"Moved to point: {point.x}, {point.y}, {point.z}")

if __name__ == '__main__':
    try:
        point_mover = PointMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
