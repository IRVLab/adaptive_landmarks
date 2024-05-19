#!/usr/bin/python3

import numpy as np

class Controller():
    def __init__(self, image_width, image_height):
        self.kps = np.array([0.001, 0.001, 0.5])  # pitch, yaw, speed kp values


        self.direction_coef = np.array([-1, -1, -1])
        self.desired_s = np.array([image_height//2, image_width//2, 0.2]) # desired pitch, yaw, and speed. Pitch and yaw in pixels, where image_width//2 and image_height//2 is the center of the screen
        self.current_s = np.array([0, 0]) # current pitch, yaw. Pitch and yaw in pixels, where image_width//2 and image_height//2 is the center of the screen

        self.start_time = None # when we receive our first IMU message from ROS, update this to the current system time
        self.current_time = None

        self.image_width = image_width
        self.image_height = image_height

        self.bumped_time = None
        self.bump_thresh = 0 # if linear acceleration is < 0, we got bumped!

        self.timeout = 5


    def update_bump(self, accel_x, msg_time):
        if accel_x < 0 and self.bumped_time is None:
            self.bumped_time = msg_time

    def run_once(self, time): # point in x, y coords

        if self.bumped_time is not None:
            pitch_output = 0
            yaw_output = 0
            if (self.bumped_time - time) < 5: # go forward for 5 seconds afterwards
                throttle_output = 0.2
            else:
                throttle_output = 0
        else:

            pitch_output = np.clip(self.kps[0] * (self.current_s[0] - self.desired_s[0]), -1, 1)
            yaw_output = np.clip(self.kps[1] * (self.current_s[1] - self.desired_s[1]), -1, 1)
            throttle_output = np.clip(self.kps[2] * self.desired_s[2], -1, 1)

        return pitch_output, yaw_output, throttle_output