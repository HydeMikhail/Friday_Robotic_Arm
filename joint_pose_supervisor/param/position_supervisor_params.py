#!/home/rosvm/vEnvs/rosPy/bin/python

'''
Motor Control Package Parameters
'''

import rospy

# Arm Lengths
rospy.set_param('mainArmLength', 7)            # mm
rospy.set_param('secArmLength', 4)             # mm

# Effector Characteristics
rospy.set_param('effectorLength', 2)           # mm
rospy.set_param('effectorAngularOffset', 0)    # deg

# Step Resolution
rospy.set_param('stepRes', 1.8)                # deg/step

# Home Angles
rospy.set_param('baseHomeAngle', 0)            # deg
rospy.set_param('mainHomeAngle', 0)            # deg
rospy.set_param('secHomeAngle', 0)             # deg
rospy.set_param('toolHomeAngle', 0)            # deg
