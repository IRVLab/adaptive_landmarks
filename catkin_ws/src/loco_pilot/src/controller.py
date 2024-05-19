#!/usr/bin/python

# This code is a part of the LoCO AUV project.
# Copyright (C) The Regents of the University of Minnesota

# Maintainer: Junaed Sattar <junaed@umn.edu> and the Interactive Robotics and Vision Laboratory

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# This code is responsible for taking a target, such as a yaw angle, a distance, or
# a thrust value and a time, and publishing mini_pilot/Command messages in order to 
# reach that point.

import time
from math import pi

import rospy
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, Transform
from mavros_msgs.srv import SetMode
from loco_pilot.msg import Command

def DEG2RAD(degs):
    return degs * (pi/180)
        
def RAD2DEG(rads):
    return rads * (180/pi)

def angle_diff(from_theta, to_theta):
    diff = to_theta - from_theta
    if (diff > pi):
        diff = diff - 2*pi
    
    if (diff < -pi):
        diff = diff + 2*pi

    return diff


class Controller(object):
    

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.cmd = rospy.Publisher('loco/command', Command, queue_size=1)
        self.current_angles = [0, 0 ,0]
	self.set_mode('STABILIZED')
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        rospy.loginfo('Controller initialized')

    def set_mode(self, mode):
        self.mode = mode

        if mode == 'MANUAL':
            self.set_manual_mode()
        elif mode == 'STABILIZED':
            self.set_stabilized_mode()
        elif mode == 'DEPTH HOLD':
            self.set_depth_hold_mode()

    def set_manual_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'MANUAL')

    def set_stabilized_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'STABILIZED')

    def set_depth_hold_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'DEPTH HOLD') # TODO This doesn't work yet, not sure why.

    def tf_callback(self, data):
        orientation = data.transforms[0].transform.rotation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_angles = [DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw)]

    # TODO support distance.
    def thrust(self, duration=1, thrust=0.25, distance=None):
        rospy.loginfo('Controller running thrust')

        timeout = time.time() + duration
        msg = Command()
        msg.throttle = thrust

        while not(time.time() > timeout):
            self.cmd.publish(msg)
            self.rate.sleep()

    # Target angles are [R,P,Y].
    def to_orientation(self, target_angles = [0,0,0], duration=None, thrust=None, tolerance=25, constraints=[True, True, True]):
        roll_target = DEG2RAD(target_angles[0])
        pitch_target = DEG2RAD(target_angles[1])
        yaw_target = DEG2RAD(target_angles[2])

        constraints = [roll_target!=0, pitch_target!=0, yaw_target!=0]
        tolerance = DEG2RAD(tolerance)

        rospy.loginfo('Controller going to target angles [RPY-deg]: %f %f %f'%(roll_target, pitch_target, yaw_target))

        if duration == None: #Using angle target
            rospy.loginfo('No duration given, so using position estimate.')
            rospy.loginfo('Constraining R(%r) P(%r) Y(%r) by %d' % (constraints[0], constraints[1], constraints[2], tolerance))
            (roll_init, pitch_init, yaw_init) = self.current_angles
            (roll_d, pitch_d, yaw_d) = [angle_diff(roll_init, roll_target), angle_diff(pitch_init, pitch_target), angle_diff(yaw_init, yaw_target)]

            rospy.loginfo('Init angles: R=%f, P=%f, Y=%f'%(RAD2DEG(roll_init),RAD2DEG(pitch_init),RAD2DEG(yaw_init)))
            rospy.loginfo('Target angles: R=%f, P=%f, Y=%f'%(RAD2DEG(roll_target),RAD2DEG(pitch_target),RAD2DEG(yaw_target)))
            rospy.loginfo('Difference: R=%f, P=%f, Y=%f'%((RAD2DEG(roll_d),RAD2DEG(pitch_d),RAD2DEG(yaw_d))))
            
            (roll, pitch, yaw) = self.current_angles

            while ((not constraints[0]) or abs(roll_d) > tolerance) or ((not constraints[1]) or abs(pitch_d) > tolerance) or ((not constraints[2]) or abs(yaw_d) > tolerance):
                rospy.loginfo('-----STEP-----')
                rospy.loginfo('Current: R=%f, P=%f, Y=%f'%(RAD2DEG(roll),RAD2DEG(pitch),RAD2DEG(yaw)))
                rospy.loginfo('Target R=%f, P=%f, Y=%f'%(RAD2DEG(roll_target),RAD2DEG(pitch_target),RAD2DEG(yaw_target)))
                rospy.loginfo('Difference: R=%f, P=%f, Y=%f'%((RAD2DEG(roll_d),RAD2DEG(pitch_d),RAD2DEG(yaw_d))))
                
                msg = Command()

                msg.roll = (roll_target!=0) * thrust
                msg.pitch = (pitch_target!=0) * thrust
                msg.yaw = (yaw_target!=0) * thrust

                self.cmd.publish(msg)
                self.rate.sleep()
                (roll, pitch, yaw) = self.current_angles
                (roll_d, pitch_d, yaw_d) = [angle_diff(roll, roll_target), angle_diff(pitch, pitch_target), angle_diff(yaw, yaw_target
                )]

            # get current angle estimate from odometry, then loop over odom until close enough.

        else: # Using duration and thrust.
            rospy.loginfo('Given duration, overriding position estimate.')
            timeout = time.time() + duration
            msg = Command()

            if pitch_target > 0:
                msg.pitch = thrust
            if yaw_target > 0:
                msg.yaw = thrust

            while not(time.time() > timeout):
                self.cmd.publish(msg)
                self.rate.sleep()

    def thurst_and_orientation(self):
        pass


    def do_square(self, sideA_length=1, sideB_length=1, thrust=0.5, right=True):
        sideA_duration = sideA_length * 5 
        sideB_duration = sideB_length * 5

        self.thrust(sideA_duration, thrust) # Leg 1
        self.to_orientation(target_angles=[0,0,1], duration=1.5, thrust=thrust)
        self.thrust(sideB_duration, thrust) # Leg 2
        self.to_orientation(target_angles=[0,0,1], duration=1.5, thrust=thrust)
        self.thrust(sideA_duration, thrust) # Leg 3
        self.to_orientation(target_angles=[0,0,1], duration=1.5, thrust=thrust)
        self.thrust(sideB_duration, thrust) # Leg 4
        self.to_orientation(target_angles=[0,0,1], duration=1.5, thrust=thrust)

    def do_circle(self, duration=20, radius=0, thrust=0.5):
        #TODO radius not yet supported.
        timeout = time.time() + duration
        msg = Command()
        msg.throttle = thrust/2
        msg.yaw = thrust

        while not(time.time() > timeout):
            self.cmd.publish(msg)
            self.rate.sleep()
