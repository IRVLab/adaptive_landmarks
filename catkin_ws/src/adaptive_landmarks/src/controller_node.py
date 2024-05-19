#!/usr/bin/python3


import rospy
from loco_pilot.msg import Command
from sensor_msgs.msg import Imu
from adaptive_landmarks.msg import Markers

from controller import Controller

class ControllerNode(Controller):
    def __init__(self):
        super().__init__(480, 640)

        rospy.init_node("controller", disable_signals=True)

        rospy.Subscriber("/mavros/imu/data_raw", Imu, self.__imu_callback, queue_size=1)  # set up our listener on the IMU topic
        rospy.Subscriber("/blob_detector/target_pt", Markers, self.__curr_callback, queue_size=1) # where the AUV currently sees the docking station at
        rospy.Subscriber("/controller/setpoint", Markers, self.__setpoint_callback, queue_size=1) # where we want to steer the AUV
        self.pubControl = rospy.Publisher("/loco/command", Command)

        self.rate = rospy.Rate(30) # 30 hz

    def __imu_callback(self, msg):

        msg_time = msg.header.stamp.to_sec()
        self.update_bump(msg.linear_acceleration.x, msg_time)

    def __curr_callback(self, msg):
        self.current_s[0] = msg.v[0]
        self.current_s[1] = msg.u[0]

    def __setpoint_callback(self, msg):

        self.desired_s[0] = msg.v[0] # desired pitch, in pixels
        self.desired_s[1] = msg.u[0] # desired yaw, in pixels
        self.desired_s[2] = 0.2


    def run_node(self):
        cmd_msg = Command()

        if self.current_s[0] != 0 or self.current_s[1] != 0:
            pitch, yaw, throttle = self.run_once(rospy.Time.now().to_sec())
            cmd_msg.pitch = pitch
            cmd_msg.yaw = yaw
            cmd_msg.throttle = throttle
        else:
            cmd_msg.pitch = 0
            cmd_msg.yaw = 0
            cmd_msg.throttle = 0

        self.pubControl.publish(cmd_msg)
        self.rate.sleep()

if __name__ == "__main__":
    bd = ControllerNode()
    while not rospy.is_shutdown():
        bd.run_node()
