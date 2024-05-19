#!/usr/bin/python3
import colorsys
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge

class Logger:
    def __init__(self, ros_enabled):
        self.ros_enabled = ros_enabled

    def log(self, msg):
        if self.ros_enabled:
            # rospy.loginfo(msg)
            pass
        else:
            # print(msg)
            pass
class BlobDetector:
    def __init__(self, ros_enabled=False):

        self.ros_enabled = ros_enabled
        self.logger = Logger(ros_enabled)
        self.cvbr = CvBridge()
        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 5
        params.filterByArea = True
        params.minArea = 4
        self.detector = cv2.SimpleBlobDetector_create(params)
        self.rgb = [0, 0, 0]
        self.gt_marker_locations_2d = np.array([
            [0.0, 0.0],
            [5.0, 0.0],
            [5.0, 35.0],
            [0.0, 35.0],
        ])
        self.marker_xy_ratio = 35.0 / 5.0
        self.marker_tolerance = .5 #  unitless tolerance for the xy ratio
        self.target_point = [0, 0]
        self.num_empty_frames = 0

    def detect(self, image, marker_rgb, max_channel=True):

        bin_img = cv2.GaussianBlur(image, (5, 5), 0)
        bins = np.array([0, 51, 102, 153, 204, 255])
        bin_img[:, :, :] = np.digitize(bin_img[:, :, :], bins, right=True) * 51
        bin_img_grey = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        bin_hist = np.unique(bin_img_grey, return_counts=True)

        max_bin_idx = np.argmax(bin_hist[1])
        max_bin_value = bin_hist[0][max_bin_idx]

        background_mask = np.where((bin_img_grey == max_bin_value), bin_img_grey, 0)
        not_background_mask = np.where((bin_img_grey != max_bin_value), bin_img_grey, 0)
        background = cv2.bitwise_and(image, image, mask=background_mask)
        not_background = cv2.bitwise_and(cv2.GaussianBlur(image, (5,5), 0), cv2.GaussianBlur(image, (5,5), 0), mask=not_background_mask)
        n_nonzero = np.count_nonzero(background_mask)

        rgb_bgr_bckgnd = (np.sum(background[:, :, 0]) / n_nonzero,
                          np.sum(background[:, :, 1]) / n_nonzero,
                          np.sum(background[:, :, 2]) / n_nonzero)

        avg_rgb = rgb_bgr_bckgnd[::-1]
        self.logger.log(f"avg_rgb: {avg_rgb}")

        hsv_image = cv2.cvtColor(not_background, cv2.COLOR_BGR2HSV)
        self.rgb = marker_rgb
        self.logger.log(f"marker rgb aka self.rgb: {self.rgb}")
        max_idx = np.argmax(np.asarray(self.rgb))

        if max_channel:
            this_rgb = [0,0,0]
            this_rgb[0] = max( avg_rgb[0], (0.5*self.rgb[0] + 0.5*avg_rgb[0]))
            this_rgb[1] = max( avg_rgb[1], (0.5*self.rgb[1] + 0.5*avg_rgb[1]))
            this_rgb[2] = max( avg_rgb[2], (0.5*self.rgb[2] + 0.5*avg_rgb[2]))
        else:
            this_rgb = marker_rgb

        self.logger.log(f"this_rgb: {this_rgb}")
        hsv_tuple = colorsys.rgb_to_hsv(*(this_rgb))
        hsv_target = [hsv_tuple[0] * 180.0, hsv_tuple[1] * 255.0, hsv_tuple[2] * 1.0]
        self.logger.log(f"hsv_target: {hsv_target}")

        # For LoCO's low light cams, the hue in the HSV needs to be about 5% higher
        hsv_lb = np.array([(hsv_target[0]-30) % 180,
                           np.clip(hsv_target[1] - 45, 0, 255),
                           np.clip(hsv_target[2] - 45, 0, 255)], dtype="uint8")
        hsv_ub = np.array([(hsv_target[0] + 30) % 180,
                           np.clip(hsv_target[1] + 45, 0, 255),
                           np.clip(hsv_target[2] + 45, 0, 255)], dtype="uint8")


        self.logger.log(f"hsv_lb: {hsv_lb} hsv_ub: {hsv_ub}")
        if hsv_lb[0] > hsv_ub[0]: # need two masks when we're crossing 180->0 ?
            mask1 = cv2.inRange(hsv_image, np.array([0, hsv_lb[1], hsv_lb[2]], dtype="uint8"), hsv_ub)
            mask2 = cv2.inRange(hsv_image, hsv_lb, np.array([180, hsv_ub[1], hsv_ub[2]], dtype="uint8"))
            mask = mask1 + mask2
            self.logger.log("flipped bounds for inrange")
        else:
            mask = cv2.inRange(hsv_image, hsv_lb, hsv_ub)

        mask = cv2.GaussianBlur(mask,(5,5), 0)
        retval, mask_inv = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY_INV)
        self.logger.log(f"mask_inv shape: {mask_inv.shape}")
        masked_im = cv2.bitwise_and(image, image, mask=mask)
        self.logger.log(f"hsv_target: {hsv_target}")

        # Detect blobs
        keypoints = list(self.detector.detect(mask_inv))
        self.logger.log(f"detected blob keypoints: {keypoints}")

        # n=8 lowest points
        if len(keypoints) > 8:
            yvals = [abs(kp.pt[1] - image.shape[0]) for kp in keypoints]
            n_remove = len(keypoints) - 8
            for i in range(n_remove):
                idx = np.argmax(yvals)
                yvals.pop(idx)
                keypoints.pop(idx)

        # Do some spatial filtering
        spatial_filtered_pts = set()

        if len(keypoints) > 0:
            # start with kp1, calculate pythagorean distance between all other points.
            for kp1 in keypoints:
                dists = np.zeros(len(keypoints))
                x1, y1 = kp1.pt
                for idx2, kp2 in enumerate(keypoints):
                    x2, y2 = kp2.pt
                    dists[idx2] = np.sqrt((x1 - x2)**2 + (y1-y2)**2)
                # now that we have the distances between all the points, calculate the ratios between all points
                dists_ratios = np.zeros((len(keypoints), len(keypoints)))
                triangle = np.zeros((len(keypoints), len(keypoints)))
                for idx2, kp2 in enumerate(keypoints):
                    x2, y2 = kp2.pt
                    for idx3, kp3 in enumerate(keypoints):
                        x3, y3 = kp3.pt
                        side_a = (x2 - x1)**2 + (y2 - y1)**2
                        side_b = (x3 - x2)**2 + (y3 - y2)**2
                        side_c = (x3 - x1)**2 + (y3 - y1)**2

                        if side_a > 0 and side_b > 0 and side_c > 0 and \
                            (np.isclose(side_a, (side_b + side_c), atol=2000) or
                             np.isclose(side_b, (side_a + side_c), atol=2000) or
                             np.isclose(side_c, (side_a + side_b), atol=2000)):
                            triangle[idx2][idx3] = 1

                # Calculate ratio between this point and two other points
                for j1, d1 in enumerate(dists):
                    for j2, d2 in enumerate(dists):
                        if d1 != 0 and d2 !=0:
                            dists_ratios[j1][j2] = d1/d2
                        else:
                            dists_ratios[j1][j2] = 0.0

                # These ratios are between every combination of two points. Now, filter out the points that have
                # the correct ratio. The x,y location in the matrix corresponds to the index of each point in the
                # keypoints array.
                matches = np.argwhere((dists_ratios <= self.marker_xy_ratio + self.marker_tolerance) &
                                      (self.marker_xy_ratio - self.marker_tolerance < dists_ratios)
                                      &
                                      (triangle > 0)
                                      )

                if matches.shape[0] > 0:
                    self.logger.log(f"found a point that fits the critera! {matches}")
                    self.logger.log(f"those points are {[(keypoints[xy[0]].pt, keypoints[xy[1]].pt) for xy in matches.tolist()]}")
                    pass

                for xy in matches.tolist():
                    spatial_filtered_pts.add(kp1)
                    spatial_filtered_pts.add(keypoints[xy[0]])
                    spatial_filtered_pts.add(keypoints[xy[1]])

        # Draw detected blobs as red circles.
        self.logger.log(f"spatial filtered pts: {[pt.pt for pt in spatial_filtered_pts]}")
        avg_arr = np.array([(int(pt.pt[0]), int(pt.pt[1])) for pt in spatial_filtered_pts])
        cob_filtered = np.rint(np.average(avg_arr, 0)).astype(np.uint16)
        self.logger.log(f"np.argwhere(mask > 0): {np.argwhere(mask > 0)}")
        self.logger.log(f"cob_filtered: {cob_filtered}")
        self.logger.log(f"cob_filtered shape: {cob_filtered.shape}")

        if len(cob_filtered.shape) < 1:
            cob_filtered = np.array([0,0])

        if cob_filtered[0] == 0 and cob_filtered[1] == 0:
            self.num_empty_frames +=1
            if self.num_empty_frames >=3:
                self.num_empty_frames = 3
                cob_filtered = np.array([image.shape[1]//2, image.shape[0]//2])
        else:
            self.num_empty_frames = 0

        if self.target_point[0] == 0 and self.target_point[1] == 0:
            self.target_point[0], self.target_point[1] = cob_filtered[0], cob_filtered[1]
        elif not (np.nan in cob_filtered) and (cob_filtered[0] != 0 or cob_filtered[1] != 0):
            self.target_point[0] += np.clip(int(cob_filtered[0])-self.target_point[0], -5, 5)
            self.target_point[1] += np.clip(int(cob_filtered[1])-self.target_point[1], -5, 5)

        self.logger.log(f"self.target_point: {self.target_point}")


        im_with_keypoints = cv2.drawKeypoints(masked_im, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, tuple(spatial_filtered_pts), np.array([]), (0, 255, 0),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return im_with_keypoints, mask, spatial_filtered_pts, not_background, bin_img_grey, self.target_point