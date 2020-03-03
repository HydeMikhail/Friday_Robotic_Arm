#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Client

Service = /coordinatePass

Object incorporated in dataPuller Client which takes
an XYZ coordiante from the data frame and coverts into
steps and angles for each of the four motors.
'''

from math import pi, atan2, acos
import rospy
from pose_supervisor.position_vector import PositionVector


def lawOfCos1(len1, len2, len3):
    '''
    Law of Cosines :: SITUATION 1 ::
    '''
    return acos((len3**2 + len1**2 - len2**2)
                               / (2 * len3 * len1))


def lawOfCos2(len1, len2, len3):
    '''
    Law of Cosines :: SITUATION 2 ::
    '''
    return acos((len1**2 + len2**2 - len3**2)
                               / (2 * len1 * len2))


class GeometryUtil(object):
    '''
    Consists of math utils for the /coordinatePass
    Service. Integrated into Service Client.
    '''

    def __init__(self):
        self.vector = PositionVector(0, 0, 0)
        self.mainArmLength = rospy.get_param('mainArmLength')
        self.secArmLength = rospy.get_param('secArmLength')

    def base_pose(self):
        '''
        Solves for the desired base angle to achieve goal position
        Returns angle in Degrees
        '''
        return atan2(self.vector.yCoor, self.vector.xCoor)

    def main_pose(self):
        '''
        Solves for the desired main arm angle to achieve goal position
        Returns angle in Degrees
        '''
        return 90 - lawOfCos1(self.mainArmLength, self.secArmLength,
                              self.vector.pvLength) - self.vector.pvAng

    def sec_pose(self):
        '''
        Solves for the desired second arm angle to achieve goal position
        Returns angle in Degrees
        '''
        return lawOfCos2(self.mainArmLength,
                         self.secArmLength,
                         self.vector.pvLength)

    def tool_pose(self):
        '''
        Compacts the separate arm angle function into one single call
        '''
        mainAng = self.main_pose()
        secAng = self.sec_pose()
        angleRemainder = pi - (mainAng + secAng)
        return pi - angleRemainder - self.vector.pvAng

############################################################
############################################################
############################################################


def coordinateErrorMsg():
    '''
    Error Message when a coordinate is outside
    the bounds of the arms capabilities.
    '''
    print '''


=========================================
        COORDINATE NOT VALID!!!
    PLEASE CORRECT THE INSTRUCTION...

    ARM RETURNING TO HOME POSITION...
=========================================


    '''
if __name__ == '__main__':
    util = GeometryUtil()
    util.vector.xCoor = 1
    util.vector.yCoor = 1
    util.vector.zCoor = 1
    print util.vector.pvAng
    print util.vector.pvLength