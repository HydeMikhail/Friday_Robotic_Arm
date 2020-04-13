#!/home/rosvm/vEnvs/rosPy/bin/python

'''
ROS Service - Server:

Feeds Coors from the CSV File to
the Data Handler upon service request.

Implements a Pandas DataFrame to contain
Coor information as well as actuator
and motion commands.
'''

import rospy
from arm_msgs.srv import instructionPass, \
    instructionPassResponse
from pandas import DataFrame, read_csv


class DataframeServer(object):
    '''
    Coor Distributor

    topic =  /instructionPass
    node  =  /instructionPassServer
    '''

    def __init__(self, filePath):
        self.robo_routine = DataFrame(read_csv(filePath, sep=","))
        self.index = 0
        self.instruction_count = 0

    def count_instructions(self):
        '''
        Counts the amount of Coors
        in routine
        '''
        self.instruction_count = self.robo_routine.shape[0]

    def check_index(self):
        '''
        Checks the current index after each service
        and sets the index back to 0 to start
        the routine over again
        '''
        print 'Index: %d' % self.index
        if self.index < self.instruction_count - 1:
            self.index += 1
        else:
            self.index = 0

    def handle_request(self, req):
        '''
        Service Handle Function:

        Requeset Message Type - Boolean
        Any message from the client will trigger the function
        '''
        if req.request:
            resp = instructionPassResponse()
            resp.motionType, resp.xCoor, resp.yCoor, resp.zCoor, \
                resp.velFac, resp.accFac = self.robo_routine.values[
                    self.index, :]

            self.check_index()

        return resp

    def run_server(self):
        '''
        Single method callable function.
        Starts the ROS Server as long as
        it is called in an initiated node
        '''
        self.count_instructions()
        rospy.Service('instructionPass', instructionPass, self.handle_request)
        rospy.spin()


if __name__ == '__main__':
    try:
        print 'Server Running ... '
        rospy.init_node('dataFrameServer')
        serv = DataframeServer(
            rospy.get_param('filePath'))
        serv.run_server()
    except rospy.ROSInterruptException():
        pass
