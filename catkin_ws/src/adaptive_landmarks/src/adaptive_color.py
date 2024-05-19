#!/usr/bin/python3
import colorsys

import cv2
import numpy as np

from cv_bridge import CvBridge

class AdaptiveColor:
    def __init__(self):

        self.cvbr = CvBridge()

        self.hue_avgs = []
        self.max_num_hue_avgs = 10
    def calc_marker_colors(self, image, marker_cc=None, shift=120):
        # Calculate the average background color by converting to greyscale and binning
        bin_img = cv2.GaussianBlur(image, (5, 5), 0)
        bins = np.array([0, 51, 102, 153, 204, 255])
        bin_img[:, :, :] = np.digitize(bin_img[:, :, :], bins, right=True) * 51
        bin_img_grey = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        bin_hist = np.unique(bin_img_grey, return_counts=True)

        max_bin_idx = np.argmax(bin_hist[1])
        max_bin_value = bin_hist[0][max_bin_idx]

        background_mask = np.where((bin_img_grey == max_bin_value), bin_img_grey, 0)
        background = cv2.bitwise_and(image, image, mask=background_mask)
        n_nonzero = np.count_nonzero(background_mask)

        rgb_bgr_bckgnd = (np.sum(background[:, :, 0]) / n_nonzero,
                          np.sum(background[:, :, 1]) / n_nonzero,
                          np.sum(background[:, :, 2]) / n_nonzero)[::-1]

        avg_hsv = colorsys.rgb_to_hsv(*(rgb_bgr_bckgnd))

        if len(self.hue_avgs) > self.max_num_hue_avgs:
            self.hue_avgs.pop(0)
        self.hue_avgs.append(avg_hsv[0])

        if shift == 120:
            hsv_marker_color = [np.average(np.asarray(self.hue_avgs)) - (2.0/3.0), 1, 255] # 120 degree shift
        elif shift == 180:
            hsv_marker_color = [np.average(np.asarray(self.hue_avgs)) - (1.0/2.0), 1, 255] # 180 degree shift

        if hsv_marker_color[0] < 0:
            hsv_marker_color[0] += 1.0
        elif hsv_marker_color[0] > 1.0:
            hsv_marker_color[0] -= 1.0

        if marker_cc is not None:
            hsv_marker_color[0] += marker_cc

        rgb_marker_color = colorsys.hsv_to_rgb(*hsv_marker_color)
        rgb_marker_color = [int(c) for c in rgb_marker_color]

        swatch_size = (512, 512, 3)
        compliment_swatch = cv2.rectangle(255 * np.ones(shape=swatch_size, dtype=np.uint8), (0, 0), swatch_size[:2], rgb_bgr_bckgnd, -1)
        color_swatch = cv2.rectangle(compliment_swatch, (int((3/8) * swatch_size[0]), int((3/8* swatch_size[1]))), (int((5/8) * swatch_size[0]), int((5/8* swatch_size[1]))), rgb_marker_color, -1)
        color_swatch = cv2.rectangle(color_swatch, (int((7/8) * swatch_size[0]), int((7/8* swatch_size[1]))), swatch_size[:2], colorsys.hsv_to_rgb(avg_hsv[0], 1, 255), -1)
        color_swatch = cv2.cvtColor(color_swatch, cv2.COLOR_BGR2RGB)

        return rgb_marker_color, color_swatch, background, bin_img_grey