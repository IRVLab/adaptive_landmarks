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

# Thanks to Corey Knutson and Hunter Bashaw for their work on the initial version of 
# the loco_teleop package, from which a lot of this code is adapted.

# This code is responsible for arming the robot, controlling the robot's mode, and transforming loco_pilot/Command 
# messages to RC controls. 

import rospy

from loco_pilot.msg import Command

from mavros_msgs.msg import OverrideRCIn as rc
from mavros_msgs.srv import StreamRate
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import mavros
import mavros.command 

pubRC = None,
rc_msg = None
neutral_speed = 1500
new_command = False

def convert_command(data):
    #This takes the float value from -1.0 to 1.0 and converts it to be between 1100 and 1900
    if(data >0.6):
	    return int((0.6 * 400) + neutral_speed)
    else:
	    return int((data * 400) + neutral_speed)

def command_callback(data):
    global pubRC, rc_msg, new_command
    new_command = True
    rc_msg = rc()
    rc_msg.channels[0] = neutral_speed
    rc_msg.channels[1] = neutral_speed
    rc_msg.channels[2] = convert_command(data.pitch)
    rc_msg.channels[3] = convert_command(data.yaw)
    rc_msg.channels[4] = convert_command(data.throttle*-1)

def set_stream_rate():
    stream_rate = rospy.ServiceProxy('mavros/set_stream_rate', StreamRate)
    stream_rate(0, 10, 1)

def set_manual_mode():
    set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/set_mode')
    set_mode(0, 'MANUAL')

def set_stabilized_mode():
    set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/set_mode')
    set_mode(0, 'STABILIZED')

def set_depth_hold_mode():
    set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/set_mode')
    set_mode(0, 'DEPTH HOLD') # TODO This doesn't work yet, not sure why.

def arm():
    mavros.set_namespace()
    mavros.command.arming(True)

def init_pixhawk():
    set_stream_rate()
#    set_manual_mode()
    set_stabilized_mode()
    arm()

def pilot():
    global pubRC, rc_msg, new_command

    rospy.init_node('pilot', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pubRC = rospy.Publisher('mavros/rc/override', rc, queue_size=10)
    rospy.Subscriber("loco/command", Command, command_callback)

    init_pixhawk()

    while not rospy.is_shutdown():
        if new_command:
            pubRC.publish(rc_msg)
            new_command = False
            rospy.loginfo('Publishing non-neutral Command')
        else:
            rc_msg = rc()
            rc_msg.channels[0] = neutral_speed
            rc_msg.channels[1] = neutral_speed
            rc_msg.channels[2] = neutral_speed
            rc_msg.channels[3] = neutral_speed
            rc_msg.channels[4] = neutral_speed
            pubRC.publish(rc_msg)
            
        rate.sleep()

    

if __name__ == '__main__':
    try:
        pilot()
    except rospy.ROSInterruptException:
        pass
