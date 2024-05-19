#!/usr/bin/python3
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from adaptive_landmarks.msg import Rgb

from adaptive_color import AdaptiveColor

class AdaptiveColorNode(AdaptiveColor):
    def __init__(self):
        super().__init__()

        rospy.init_node("adaptive_color", disable_signals=True)


        self.sub = rospy.Subscriber(rospy.get_param('/adaptive_color_node/camera_topic'), Image, self.image_callback)

        self.pub_swatch = rospy.Publisher("/adaptive_color/swatch", Image, queue_size=1)
        self.pub_background_masked = rospy.Publisher("/adaptive_color/background_masked", Image, queue_size=1)
        self.pub_bin_img_grey = rospy.Publisher("/adaptive_color/bin_img_grey", Image, queue_size=1)
        self.pub_rgb = rospy.Publisher("/marker_color/calculated_color", Rgb, queue_size=1)

        self.cvbr = CvBridge()

        self.image = None

        self.rate = rospy.Rate(5) # 5hz

        self.hue_avgs = []
        self.max_num_hue_avgs = 10
    def image_callback(self, msg: Image):
        self.image = self.cvbr.imgmsg_to_cv2(msg)


    def run_node(self):
        if self.image is not None:
            rgb_marker_color, color_swatch, background, bin_img_grey = self.calc_marker_colors(self.image, shift=120) # Set shift=180 for complementary markers

            rgb_msg = Rgb()
            rgb_msg.r = rgb_marker_color[0]
            rgb_msg.g = rgb_marker_color[1]
            rgb_msg.b = rgb_marker_color[2]
            self.pub_rgb.publish(rgb_msg)

            self.pub_swatch.publish(self.cvbr.cv2_to_imgmsg(color_swatch, encoding="bgr8"))
            self.pub_background_masked.publish(self.cvbr.cv2_to_imgmsg(background, encoding="bgr8"))
            self.pub_bin_img_grey.publish(self.cvbr.cv2_to_imgmsg(bin_img_grey))

        self.rate.sleep()

if __name__ == "__main__":
    mc = AdaptiveColorNode()
    while not rospy.is_shutdown():
        mc.run_node()