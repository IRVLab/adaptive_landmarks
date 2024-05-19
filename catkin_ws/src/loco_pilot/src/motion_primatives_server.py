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

# This code is responsible for advertising services which expose primative motions
# to the user, such as Forward motion, Square, Circle, etc.

import rospy

from loco_pilot.srv import Thrust, Pitch, Yaw, Square, Circle, Saw
from controller import Controller


controller = Controller()

def thrust_handler(req):
    duration = req.duration
    thrust = req.thrust
    rospy.loginfo('Recieved thrust request: duration=%d, thrust=%f'%(duration, thrust))
    #distance = req.distance #TODO Support distance.

    rospy.loginfo('Sending thrust request to controller.')
    controller.thrust(duration, thrust)
    return True

def pitch_handler(req):
    duration = req.duration
    thrust = req.thrust
    pitch = req.angle
    rospy.loginfo('Recieved pitch request: duration=%d, thrust=%f, pitch=%f'%(duration, thrust, pitch))

    if duration == 0:
        controller.to_orientation([0, pitch, 0], duration=None, thrust=thrust, tolerance=10)
    else:
        controller.to_orientation([0, 1, 0], duration=duration, thrust=thrust)

    return True
    

def yaw_handler(req):
    duration = req.duration
    thrust = req.thrust
    yaw = req.angle
    rospy.loginfo('Recieved yaw request: duration=%d, thrust=%f, yaw=%f'%(duration, thrust, yaw))

    if duration == 0:
        controller.to_orientation([0, 0, yaw], duration=None, thrust=thrust, tolerance=10)
    else:
        controller.to_orientation([0, 0, 1], duration=duration, thrust=thrust)

    return True

def square_handler(req):
    sideA = req.sideA
    sideB = req.sideB
    thrust = req.thrust
    right = req.right

    rospy.loginfo('Recieved square request: sideA=%d, sideB=%d, thrust=%f, right square?=%r'%(sideA, sideB, thrust, right))
    controller.do_square(sideA, sideB, thrust, right)
    
    return True

def circle_handler(req):
    duration = req.duration
    radius = req.radius
    thrust = req.thrust

    rospy.loginfo('Recieved circle request: duration=%f, radius=%d, thrust=%f'%(duration, radius, thrust))
    controller.do_circle(duration=duration, radius=radius, thrust=thrust)
    

if __name__ == "__main__":
    rospy.loginfo('Initializing Motion Primatives server...')

    rospy.Service('/loco/controller/thrust', Thrust, thrust_handler)
    rospy.Service('/loco/controller/pitch', Pitch, pitch_handler)
    rospy.Service('/loco/controller/yaw', Yaw, yaw_handler)
    rospy.Service('/loco/controller/square', Square, square_handler)
    rospy.Service('/loco/controller/circle', Circle, circle_handler)


    rospy.loginfo('Service advertising completed...')
    rospy.loginfo('Motion primatives server ready for business!')
    rospy.loginfo('Spinning forever until a service request is recieved.')    

    # Spin forever to avoid early shutdown.
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        
else:
    pass
