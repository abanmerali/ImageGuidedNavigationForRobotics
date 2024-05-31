#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    else:
        raise TypeError("Must be of type list")

    return True

def is_pose_reachable(pose_goal):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pose_reachability_checker', anonymous=True)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "main_planning_group"
    move_group = MoveGroupCommander(group_name)

    move_group.set_pose_target(pose_goal)
    plan = move_group.plan()

    if plan:
        rospy.loginfo("Pose is reachable.")
        return True
    else:
        rospy.logwarn("Pose is not reachable.")
        return False

if __name__ == '__main__':
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4  # Adjust these values
    pose_goal.position.y = 0.1  # Adjust these values
    pose_goal.position.z = 0.4  # Adjust these values

    try:
        if is_pose_reachable(pose_goal):
            rospy.loginfo("The pose is reachable.")
        else:
            rospy.logwarn("The pose is not reachable.")
    except rospy.ROSInterruptException:
        pass


