#!/home/rosvm/vEnvs/rosPy/bin/python

'''
This is a script which utilizes the MoveIt
Python API to demo the use of Python as a
a controller for a ROS MoveIt Stack.
'''

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#Initializes ROS Node and MoveIt Commander (MoveIt API)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('python_move_group', anonymous=True)

#Robot Commander is the outer-level interface to the robot
friday = moveit_commander.RobotCommander()

#Initiate the Planning Scene which interfaces to the world around
#around the robot
scene = moveit_commander.PlanningSceneInterface()

#Initiate the Move Group Commander which controls a joint group
group_name = 'Friday_ARM'
group = moveit_commander.MoveGroupCommander(group_name)

#Create a publisher to send trajectory display data to RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

#PLANNING A JOINT GOAL - Position by Joint Angles

def joint_goal_DEMO(angles):
    '''
    Creates a motion plan by defining the angles for
    each joint in the MoveIt Stack
    '''
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = angles[0]
    joint_goal[1] = angles[1]
    joint_goal[2] = angles[2]

    group.go(joint_goal, wait=True)

    group.stop()

#PLANNING A POSE GOAL

def pose_goal_DEMO(orientation):
    '''
    Creates a motion plan by defining the cartesian
    coordinates of the end effector
    '''
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()