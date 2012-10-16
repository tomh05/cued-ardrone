#!/usr/bin/env python

# A simple example of doing frame-to-frame feature matching using the
# fundamental matrix constraint.

import cv2
import numpy as np
import roslib
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class FeatureMatcher(object):
    def __init__(self):
        cv2.namedWindow('Output')

        # A tuple of image, features and descriptors
        self._prev = self._this = None
        self._matching = None

        # Empirically the ORB extractor is by far the fastest.
        self._feature_detector = cv2.FeatureDetector_create('ORB')
        self._descriptor_extractor = cv2.DescriptorExtractor_create('ORB')
        self._descriptor_matcher = cv2.DescriptorMatcher_create('BruteForce')

    def new_frame(self, image):
        if image.encoding != 'rgb8':
            rospy.logwarn('Unknown image encoding: %s' % (image.encoding,))
        
        colour_im = np.reshape(np.frombuffer(image.data, dtype=np.uint8), (image.height, image.width, 3))
        im = cv2.cvtColor(colour_im, cv2.COLOR_BGR2GRAY)

        self._prev = self._this
        self._this = (im,) + self._extract_features_and_descriptors(im)

        self._update()

        out = cv2.cvtColor(colour_im, cv2.COLOR_BGR2RGB)
        if self._matching:
            for p1, p2 in zip(*self._matching[1:3]):
                p1 = tuple(int(x) for x in p1)
                p2 = tuple(int(x) for x in p2)
                cv2.line(out, p1, p2, (0,200,0))
                cv2.circle(out, p1, 3, (0,0,200))
                cv2.circle(out, p2, 3, (200,0,0))

        cv2.imshow('Output', out)
        cv2.waitKey(1)

    def _extract_features_and_descriptors(self, image):
        keypoints = self._feature_detector.detect(image)
        keypoints, descriptors = self._descriptor_extractor.compute(image, keypoints)
        pts = np.array(list(x.pt for x in keypoints))
        return pts, descriptors, keypoints

    def _update(self):
        self._matching = None
        if self._this is None or self._prev is None:
            return
        if self._this[2] is None or self._prev[2] is None:
            return
        if len(self._this[2]) < 1 or len(self._prev[2]) < 1:
            return

        matches = self._descriptor_matcher.match(self._this[2], self._prev[2])
        if not matches or len(matches) < 8:
            return

        prev_pts = self._prev[1][list(x.trainIdx for x in matches), :]
        this_pts = self._this[1][list(x.queryIdx for x in matches), :]

        H, mask = cv2.findFundamentalMat(prev_pts, this_pts, cv2.FM_RANSAC, 3.0, 0.99)

        self._matching = (H, np.compress(mask.flat, prev_pts, 0), np.compress(mask.flat, this_pts, 0), matches, mask)

def run():
    m = FeatureMatcher()

    rospy.init_node('xbox_controller')
    rospy.Subscriber('/ardrone/front/image_raw', Image, m.new_frame)
    rospy.spin()

if __name__ == '__main__':
    run()

# vim:sw=4:sts=4:et
