#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Position Vector
'''

from math import atan, pi, sqrt


class PositionVector(object):
    '''
    Properties of the vector defining the destination
    of the arm.
    '''

    def __init__(self, xCoor, yCoor, zCoor):
        self.xCoor = xCoor
        self.yCoor = yCoor
        self.zCoor = zCoor
        self.pvAng = 0
        self.pvLength = 0

    def vector_check(self):
        '''
        Returns True if the arm can achieve the components
        of the position vector. This will also avoid any
        runtime errors of calculating undefined values. The
        domain is determined by the length of the arms.
        '''
        comparisonValue = sqrt(self.xCoor**2 + self.yCoor**2 + self.zCoor**2)
        return 2 <= comparisonValue <= 10

    def define_vector(self):
        '''
        Input the x, y, z coordinate of the desired poisiton
        and the necessary components of that vector will be
        calculated and returned. xPrime describes the horizontal
        length of the 2D vector residing in the plane already
        achieved by rotating the base.
        '''
        xPrime = sqrt(self.xCoor**2 + self.yCoor**2)
        self.pvAng = (atan(self.zCoor / xPrime)) * 180 / pi
        self.pvLength = sqrt(xPrime**2 + self.zCoor**2)
