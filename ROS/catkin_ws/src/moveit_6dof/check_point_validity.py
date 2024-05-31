#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg

def is_point_valid(pose_goal):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('check_pose_validity', anonymous=True)

    robot = moveit_commander.RobotCommander()
    group_name = "main_planning_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    waypoints = []
    waypoints.append(pose_goal)

    (plan, fraction) = move_group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

    if fraction == 1.0:
        rospy.loginfo("The point is valid and reachable.")
        return True
    else:
        rospy.logwarn("The point is not reachable.")
        return False

if __name__ == '__main__':
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5  # Adjust these values
    pose_goal.position.y = -5.0  # Adjust these values
    pose_goal.position.z = 10.0  # Adjust these values

    try:
        valid = is_point_valid(pose_goal)
        if valid:
            rospy.loginfo("The pose is valid.")
        else:
            rospy.logwarn("The pose is invalid.")
    except rospy.ROSInterruptException:
        pass
