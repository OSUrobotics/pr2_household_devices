#!/usr/bin/env python
#
# Provides service to find a device as follows:
#  1) User selects region
#  2) Filter point cloud to roughly include region (in camera frame)
#  3) Find & return plane parameters from this point cloud via another service

import roslib
roslib.load_manifest('find_devices')

from find_devices.user_tools import *  # helper scripts from "user_tools.py"
from find_devices.srv import *

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


class CloudFilterByRegion():
    """ Class to get PointCloud2 msg and filter it by a user-defined region. """
    def __init__(self, region, topic):
        self.region = region
        self.need_pointcloud = True
        rospy.Subscriber(topic, PointCloud2, self.callback_PointCloud2)
    
    def callback_PointCloud2(self, cloud):
        """ Find plane from 5cm cube of PointCloud2 msg """
        if self.need_pointcloud:
            rospy.loginfo('Got a PointCloud2!')
            self.cloud_full = cloud  # save full point cloud for KnobFinder class
            self.need_pointcloud = False

    def filter_cloud_by_region(self):
        """ Filter point cloud by user-selected region in camera frame """
        region = self.region  # for convenience
        region = [region[0][0], region[0][1], region[1][0], region[1][1]]  # flatten list
        limits = self.get_extents_of_points(region)
        filter_cloud_by_axes = rospy.ServiceProxy('filter_cloud_by_axes', FilterCloudByAxes)
        request = FilterCloudByAxesRequest()
        request.cloud_in = self.cloud_full
        request.limits_x = limits['x']
        request.limits_y = limits['y']
        request.limits_z = limits['z']
        response = filter_cloud_by_axes(request)
        self.cloud_filtered = response.cloud_out

    def get_extents_of_points(self, points):
        """ Calculate x, y, and z extents of a list of Point messages """
        first = True
        for pt in points:
            pt = pt.point  # shortcut
            if first:
                limits = {'x': [pt.x, pt.x], 'y': [pt.y, pt.y], 'z': [pt.z, pt.z]}
                first = False
            else:
                for dim in limits.keys():  # iterates over the axes: ['x', 'y', 'z']
                    limits[dim][0] = min(limits[dim][0], pt.__getattribute__(dim))  # update minimum
                    limits[dim][1] = max(limits[dim][1], pt.__getattribute__(dim))  # update maximum
        limits['z'] = [-10, 10]  # do not filter in Z-direction...yet
        return limits


class PlaneFinder():
    """ Gets plane parameters for filtered point cloud. """
    def __init__(self):
        pass

    def find_plane_from_cloud(self, cloud):
        """ """
        # Find plane from point cloud
        fit_plane = rospy.ServiceProxy('fit_plane', FitPlane)
        request = FitPlaneRequest()
        request.cloud = cloud
        response = fit_plane(request)
        coeffs_list = [response.a, response.b, response.c, response.d]
        normal_length = sqrt(sum([coeff**2 for coeff in coeffs_list[:3]]))
        self.normal = [coeff/normal_length for coeff in coeffs_list]  # includes all four coefficients

    def tf_from_plane(self, point, normal):
        """ Finds TF frame based on plane normal and a point. INPUTS: reference position (PointStamped), normal [x, y, z] """
        frame = TransformStamped()
        frame.header.frame_id = point.header.frame_id
        frame.transform.translation.x = point.point.x
        frame.transform.translation.y = point.point.y
        frame.transform.translation.z = point.point.z
        
        normal = -1 * numpy.asarray(normal)  # convert knob normal vector to numpy array
        k = numpy.array([0, 0, 1])  # z-axis 
        q = self.quaternion_from_directions(normal, k)  # get quaternion to rotate normal to z-axis
        
        frame.transform.rotation.x = q[0]
        frame.transform.rotation.y = q[1]
        frame.transform.rotation.z = q[2]
        frame.transform.rotation.w = q[3]

        return frame

    def quaternion_from_directions(self, v_from, v_to):
        """ Given two vectors as lists, constructs a numpy quaternion of form [x, y, z, w] 
        that rotates from first input 'v_from' to second input 'v_to' """
        v_from = numpy.asarray(v_from)  # convert list to numpy array
        v_from /= numpy.linalg.norm(v_from)  # normalize

        v_to = numpy.asarray(v_to)  # convert list to numpy array
        v_to /= numpy.linalg.norm(v_to)  # normalize

        u = numpy.cross(v_to, v_from)  # axis of rotation                                                                                                                                                
        theta_sin = numpy.linalg.norm(u)
        theta_cos = numpy.dot(v_to, v_from)
        theta = atan2(theta_sin, theta_cos)  # angle of rotation 
        u /= numpy.linalg.norm(u)  # axis of rotation, normalized                                                                                                                                         
        
        # Convert to quaternion notation 
        w = cos(0.5 * theta)
        xyz = sin(0.5 * theta) * u
        
        return numpy.array([xyz[0], xyz[1], xyz[2], w])  # quaternion in numpy array format


def find_device(request):
    """ Service callback to find a device. """
    
    ######## Get 2D region from user ###########
    grab_reg = Grabber('/head_mount_kinect/rgb/image_rect_color',  # image topic
                       'Draw a Rectangle around a Flat Part of the Device')  # window name
    while grab_reg.need_region:
        grab_reg.need_image = True
        while grab_reg.need_image:
            rospy.sleep(0.01)
        bridge = CvBridge()
        grab_reg.image = bridge.imgmsg_to_cv( grab_reg.image )
        grab_reg.image = numpy.asarray( grab_reg.image )
        cv2.imshow(grab_reg.window, grab_reg.image)  # display image
        cv.SetMouseCallback(grab_reg.window, grab_reg.callback_region)  # get mouse input
        cv.WaitKey(0)

    ####### Convert region to 3D ############
    point_from_pixel = PointFromPixel('/head_mount_kinect/depth_registered/camera_info', 
                                      '/head_mount_kinect/depth_registered/image_rect')
    while point_from_pixel.need_camera_info or point_from_pixel.need_depth_image:
        rospy.sleep(0.01)
    region = [[], []]
    for i in range(len(grab_reg.region)):
        for j in range(len(grab_reg.region[i])):
            region[i].append(point_from_pixel.calculate_3d_point(grab_reg.region[i][j]))  # get 3d point that corresponds to pixel

    ####### Filter cloud based on region ########
    cloud_filter = CloudFilterByRegion(region, '/head_mount_kinect/depth_registered/points')
    while cloud_filter.need_pointcloud:
        rospy.sleep(0.01)
    cloud_filter.filter_cloud_by_region()

    ######## Get device plane from filtered cloud #########
    plane = PlaneFinder()
    plane.find_plane_from_cloud(cloud_filter.cloud_filtered)  # find plane from filtered cloud
    frame = plane.tf_from_plane(region[0][0], plane.normal[:3])

    response = FindDeviceResponse()
    response.device = frame
    response.plane_coefficients = plane.normal
    return response


if __name__ == '__main__':

    rospy.init_node('find_device')
    srv = rospy.Service('find_device', FindDevice, find_device)
    srv.spin()
