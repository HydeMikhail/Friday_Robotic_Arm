#!/home/mhyde/vEnvs/rosPy/python

'''
ROS Service - Client:

Requests coordinates from the CSV File
to the Data Distributer upon service request.

'''

import sys
import rospy
from arm_msgs.srv import instructionPass


class DataframeClient(object):
    '''
    Client for the /instructionPass Service.
    Perpetually pulls coordinates from the dataFrame
    and feeds them to the motorPosePublisher
    '''

    def req_coordinate(self, request):
        '''
        Communicates with the data frame server and
        request one coordinate at a time. This coordinate
        is then returned as a list.
        '''
        rospy.wait_for_service('instructionPass')
        try:
            srvReq = rospy.ServiceProxy('instructionPass', instructionPass)
            resp = srvReq(request)
            if resp is not None:
                return resp
            else:
                rospy.loginfo('Server Responded with a NULL Instruction')
                return 'H', 0, 0, 0, 0, 1, 1
        except rospy.ServiceException, error:
            print "Service call failed: %s" % error

    def usage(self):
        '''
        Simple Error Check in Client Request
        '''
        return '%s [req]' % sys.argv[0]

    def client(self):
        '''
        Client Implementation
        '''
        req = False
        if len(sys.argv) <= 3:
            req = True
        else:
            print self.usage()
        print "Requesting Coordinate ... "
        coor = self.req_coordinate(req)
        return coor.motionType, coor.xCoor, coor.yCoor, coor.zCoor, \
            coor.velFac, coor.accFac


if __name__ == '__main__':
    client = DataframeClient()
    print client.client()
