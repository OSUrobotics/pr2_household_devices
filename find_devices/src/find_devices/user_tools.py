#!/usr/bin/env python
#
# Provides scripts for the following:
#  -> User selecting a point or region in image
#  -> Converting user-selected pixel(s) to 3D coordinates


import rospy
import cv
import cv2
from cv_bridge import CvBridge
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped, TransformStamped
import numpy
from image_geometry import PinholeCameraModel
from PIL import Image as img
from StringIO import StringIO
from math import sqrt, sin, cos, atan2, floor, pi


class Grabber():
    """ Have user select region in 2D image """
    def __init__(self, topic, window):
        self.window = window
        self.need_image = True
        self.need_point = True
        self.need_region = True
        self.listener = tf.TransformListener()
        rospy.Subscriber(topic, Image, self.callback_image)

    def callback_image(self, image):
        """ Get first image msg from topic """
        if self.need_image:
            self.image = image
            rospy.loginfo('Got an image!')
            self.need_image = False
            self.outlining = False

    def callback_point(self, event, u, v, flags, param):
        """ Point selection: Click to choose a point """
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.point = [u, v]
            rospy.loginfo('Got a point!')
            #image_cv2_copy = numpy.copy( self.image )
            self.image = numpy.copy( self.image )
            #cv2.circle(image_cv2_copy, tuple(self.point), 5, (0, 255, 0), -1)  # draw selected point on image
            cv2.circle(self.image, tuple(self.point), 5, (0, 255, 0), -1)  # draw selected point on image
            #cv2.imshow(self.window, image_cv2_copy)  # display for review
            cv2.imshow(self.window, self.image)  # display for review
            self.need_point = False

    def callback_region(self, event, u, v, flags, param):
        """ Region selection: Click to choose start, drag and release to choose end """
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.region = [[[u, v], [u, v]],
                           [[u, v], [u, v]]]
            rospy.loginfo('Got region start!')
            self.outlining = True
        elif event == cv.CV_EVENT_MOUSEMOVE and self.outlining:
            self.region[0][1][0] = u
            self.region[1][0][1] = v
            self.region[1][1] = [u, v]
            image_cv2_copy = numpy.copy( self.image )
            cv2.rectangle(image_cv2_copy, tuple(self.region[0][0]), tuple(self.region[1][1]), (0, 255, 0), 1)
            cv2.imshow(self.window, image_cv2_copy)  # display for review
        elif event == cv.CV_EVENT_LBUTTONUP:
            self.outlining = False
            rospy.loginfo('Got region end!')
            self.need_region = False


class PointFromPixel():
    """ Given a pixel location, find its 3D location in the world """
    def __init__(self, topic_camera_info, topic_depth):
        self.need_camera_info = True
        self.need_depth_image = True
        self.model = PinholeCameraModel()
        rospy.Subscriber(topic_camera_info, CameraInfo, self.callback_camera_info)
        rospy.Subscriber(topic_depth, Image, self.callback_depth_image)

    def callback_camera_info(self, info):
        """ Define Pinhole Camera Model parameters using camera info msg """
        if self.need_camera_info:
            rospy.loginfo('Got camera info!')
            self.model.fromCameraInfo(info)  # define model params
            self.frame = info.header.frame_id
            self.need_camera_info = False

    def callback_depth_image(self, depth_image):
        """ Get depth at chosen pixel using depth image """
        if self.need_depth_image:
            rospy.loginfo('Got depth image!')
            self.depth = img.fromstring("F", (depth_image.width, depth_image.height), depth_image.data)
            self.need_depth_image = False

    def calculate_3d_point(self, pixel):
        """ Project ray through chosen pixel, then use pixel depth to get 3d point """
        lookup = self.depth.load()
        depth = lookup[pixel[0], pixel[1]]  # lookup pixel in depth image
        ray = self.model.projectPixelTo3dRay(tuple(pixel))  # get 3d ray of unit length through desired pixel
        ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
        pt = [el * depth for el in ray_z]  # multiply the ray by the depth; its Z-component should now equal the depth value
        point = PointStamped()
        point.header.frame_id = self.frame
        point.point.x = pt[0]
        point.point.y = pt[1]
        point.point.z = pt[2]
        return point

    def ray_plane_intersection(self, pixel, plane):
        """ Given plane parameters [a, b, c, d] as in ax+by+cz+d=0,
        finds intersection of 3D ray with the plane. """ 
        ray = self.model.projectPixelTo3dRay(tuple(pixel))  # get 3d ray of unit length through desired pixel
        scale = -plane[3] / (plane[0]*ray[0] + plane[1]*ray[1] + plane[2]*ray[2])

        point = PointStamped()
        point.header.frame_id = self.frame
        point.point.x = ray[0] * scale
        point.point.y = ray[1] * scale
        point.point.z = ray[2] * scale
        return point

