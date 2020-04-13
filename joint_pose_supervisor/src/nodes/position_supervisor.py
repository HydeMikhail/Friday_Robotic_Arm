#!/home/rosvm/vEnvs/rosPy/bin/python

'''
node = /motorPosePub
pubTopic = /motorPoseSteps
subTopic = /arduinoState

Listens to arduino for ready state then implements
other methods to pull and convert the next coordinate
into the respective arm angles and steps.
'''

import rospy
from std_msgs.msg import Bool
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from arm_data_frame.data_frame_client import DataframeClient
from pose_supervisor.geometry_util import GeometryUtil


class PositionSupervisor(object):
    '''
    Pulls coordinates from the distributer and updates
    the current desired position for the arduino node.
    '''

    def __init__(self):

        self.instruction = []

        self.instructionClient = DataframeClient()
        self.geometryUtil = GeometryUtil()

        self.commander = moveit_commander.MoveGroupCommander("friday_arm")
        self.joint_goals = []

        self.stateSub = rospy.Subscriber(
            'arduinoState', Bool, callback=self.arduino_state_callback)

        rospy.spin()

    def pull_instruction(self):
        '''
        Pulls the next instruction from the data frame
        and appends it to a class member for other functions
        to access.
        '''
        self.instruction = self.instructionClient.client()

    def set_motion_type(self):
        '''
        Updates ROS motionType param to define the current
        robot motion mode.
        '''
        rospy.set_param('motionType', self.instruction[0])

    def update_position_vector(self):
        '''
        Updates the coordinates of the position vector
        for motor angle and step calculations.
        '''
        self.geometryUtil.vector.xCoor = self.instruction[1]
        self.geometryUtil.vector.yCoor = self.instruction[2]
        self.geometryUtil.vector.zCoor = self.instruction[3]
        self.geometryUtil.vector.define_vector()

    def set_velacc(self):
        '''
        Sets ROS params which scale the velocity and
        acceleration of the motors. This number is a
        percentage of the default (max) value.
        '''
        rospy.set_param('velocityFactor', self.instruction[4])
        rospy.set_param('accelerationFactor', self.instruction[5])

    def set_moveit_pose(self):
        '''
        Uses geometryUtil to return the equivalent
        angles of each motor
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.commander.set_pose_target(pose_goal)

    def submit_command(self):
        '''
        Commands the move group to start positioning
        '''
        self.commander.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.commander.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.commander.clear_pose_targets()

    def arduino_state_callback(self, arduinoState):
        '''
        Updates coordinate when Arduino is ready.
        '''
        if arduinoState.data:
            self.pull_instruction()
            self.set_motion_type()
            self.update_position_vector()
            self.set_velacc()
            self.set_moveit_pose()
            self.submit_command()

if __name__ == '__main__':
    rospy.init_node('positionSupervisor', anonymous=True)
    PositionSupervisor()
