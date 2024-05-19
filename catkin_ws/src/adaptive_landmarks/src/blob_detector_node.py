#!/usr/bin/python3
import time
import numpy as np
import rospy

from sensor_msgs.msg import Image
from adaptive_landmarks.msg import Rgb
from adaptive_landmarks.msg import Markers

from blob_detector import BlobDetector

class BlobDetectorNode(BlobDetector):
    def __init__(self):
        super().__init__(ros_enabled=True)

        rospy.init_node("blob_detector", disable_signals=True)

        self.sub_name = rospy.get_param('/blob_detector_node/camera_topic')
        time.sleep(3)

        self.sub_image = rospy.Subscriber(self.sub_name, Image, self.image_callback)
        self.sub_marker = rospy.Subscriber("/marker_color/calculated_color", Rgb, self.rgb_callback)

        self.pub_debug = rospy.Publisher("/blob_detector/image", Image, queue_size=1)
        self.pub_color_corrected = rospy.Publisher("/blob_detector/color_corrected", Image, queue_size=1)
        self.pub_target_pt = rospy.Publisher("/blob_detector/target_pt", Markers, queue_size=1)
        self.rate = rospy.Rate(30) # 30hz

        self.logger.log("Started ROS nodes")

        self.image = None

        self.rgb = [0, 0, 0]

        self.gt_marker_locations_2d = np.array([
            [0.0, 0.0],
            [5.0, 0.0],
            [5.0, 35.0],
            [0.0, 35.0],
        ])
    def rgb_callback(self, msg: Rgb):
        self.rgb[0], self.rgb[1], self.rgb[2] = msg.r, msg.g, msg.b
        self.logger.log(f"got rgb value in blob callback: {self.rgb}")
    def image_callback(self, msg: Image):
        self.image = self.cvbr.imgmsg_to_cv2(msg)

    def run_node(self):
        if self.image is not None:
            im_with_keypoints, mask, spatial_filtered_pts, _, _, target_point = self.detect(self.image, self.rgb)
            self.pub_debug.publish(self.cvbr.cv2_to_imgmsg(im_with_keypoints, encoding="bgr8"))

            mark = Markers()
            mark.u.append(target_point[0])
            mark.v.append(target_point[1])
            self.pub_target_pt.publish(mark)

        self.rate.sleep()

if __name__ == "__main__":
    bd = BlobDetectorNode()
    while not rospy.is_shutdown():
        bd.run_node()