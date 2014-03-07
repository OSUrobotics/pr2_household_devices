#!/usr/bin/env python

import roslib
roslib.load_manifest('find_devices')

import rospy

from find_devices.user_tools import *  # helper scripts from "user_tools.py"
from find_devices.srv import *


class KnobFinder():
    """ Does all final processing to get knob location, diameter, etc. """
    def __init__(self, topic, frame):
        self.frame_new = frame
        self.need_pointcloud = True
        rospy.Subscriber(topic, PointCloud2, self.callback_PointCloud2)

    def callback_PointCloud2(self, cloud):
        self.cloud_full = cloud
        self.need_pointcloud = False

    def find_knob(self, region):
        """ Get point cloud containing only the knob """
        cloud_filtered1 = self.filter_by_region(self.cloud_full, region)
        cloud_filtered2 = self.filter_by_plane(cloud_filtered1, 0.01)
        return cloud_filtered2

    def get_knob_params(self, cloud):
        """ Use knob points to get its center, diameter, and front location """
        get_knob_params = rospy.ServiceProxy('get_knob_params', GetKnobParams)
        response = get_knob_params(cloud)
        
        # Filter cloud to only get front face of knob
        cloud_knob_face = self.filter_by_plane(cloud, response.top - 0.01)

        # Re-evaluate knob parameters
        response = get_knob_params(cloud_knob_face)
        self.knob_center = response.center
        self.knob_diameter = response.diameter
        self.knob_height = response.top

        #find_cylinder = rospy.ServiceProxy('find_cylinder', FindCylinder)
        #response = find_cylinder(cloud)
        #rospy.loginfo(response)
        #rospy.loginfo(type(response.center_x))
        #self.knob_center = [response.center_x,
        #                    response.center_y,
        #                    response.center_z]
        #self.knob_diameter = response.diameter

    def filter_by_region(self, cloud, region):
        """ Filter point cloud by user-selected region in camera frame """
        region = [region[0][0], region[0][1], region[1][0], region[1][1]]
        limits = self.get_extents_of_points(region)
        filter_cloud = rospy.ServiceProxy('filter_cloud_by_axes', FilterCloudByAxes)
        request = FilterCloudByAxesRequest()
        request.cloud_in = cloud
        request.limits_x = limits['x']
        request.limits_y = limits['y']
        request.limits_z = limits['z']
        response = filter_cloud(request)
        return response.cloud_out

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
                
    def filter_by_plane(self, cloud, cutoff):
        """ Filter point cloud in device frame to remove device plane """
        cloud_device = self.transform_point_cloud(cloud, self.frame_new)  # transform point cloud to device frame
        filter_cloud = rospy.ServiceProxy('filter_cloud_by_axes', FilterCloudByAxes)
        request = FilterCloudByAxesRequest()
        request.cloud_in = cloud_device
        request.limits_x = [-10, 10]
        request.limits_y = [-10, 10]
        request.limits_z = [cutoff, 10]  # keep Z-values greater than 'cutoff'
        response = filter_cloud(request)
        return response.cloud_out

    def transform_point_cloud(self, cloud, frame):
        """ Transform point cloud into device frame """
        transform_cloud = rospy.ServiceProxy('transform_PointCloud2', TransformPointCloud2)
        request = TransformPointCloud2Request()
        request.cloud_in = cloud
        request.frame_new = frame
        response = transform_cloud(request)
        return response.cloud_out


def find_knob(request):
    """ Handles service call to find a knob. """

    ######## Get knob region from user ###########
    grab_reg = Grabber('/head_mount_kinect/rgb/image_rect_color',
                       'Draw a Rectangle around the Knob')
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

    ######## Process point cloud #########
    # Get region points
    point_from_pixel = PointFromPixel('/head_mount_kinect/depth_registered/camera_info', 
                                      '/head_mount_kinect/depth_registered/image_rect')
    while point_from_pixel.need_camera_info:
        rospy.sleep(0.01)
    region = [[], []]
    for i in range(len(grab_reg.region)):
        for j in range(len(grab_reg.region[i])):
            region[i].append(point_from_pixel.ray_plane_intersection(grab_reg.region[i][j],
                                                                     request.plane_coefficients))
         
    # Filter cloud
    knob_finder = KnobFinder('/head_mount_kinect/depth_registered/points', request.device)
    while knob_finder.need_pointcloud:
        rospy.sleep(0.01)
    cloud_knob = knob_finder.find_knob(region)

    # Get knob parameters
    knob_finder.get_knob_params(cloud_knob)

    # Build knob tf frame
    frame = TransformStamped()
    frame.header.frame_id = request.device
    frame.transform.translation.x = knob_finder.knob_center[0]
    frame.transform.translation.y = knob_finder.knob_center[1]
    frame.transform.translation.z = knob_finder.knob_height
    
    frame.transform.rotation.x = 0
    frame.transform.rotation.y = 0
    frame.transform.rotation.z = 0
    frame.transform.rotation.w = 1

    # Respond to service request
    response = FindKnobResponse()
    response.frame = frame
    response.diameter = knob_finder.knob_diameter
    response.height = knob_finder.knob_height
    return response


if __name__ == '__main__':

    rospy.init_node('find_knob')
    srv = rospy.Service('find_knob', FindKnob, find_knob)
    srv.spin()
    




