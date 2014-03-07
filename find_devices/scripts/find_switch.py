#!/usr/bin/env python

import roslib
roslib.load_manifest('find_devices')

import rospy

from find_devices.user_tools import *  # helper scripts from "user_tools.py"
from find_devices.srv import *


def make_point_tf(frame_old, pt):
    """ Make a TF frame at a point """
    frame = TransformStamped()
    frame.header.frame_id = frame_old
    frame.transform.translation.x = pt.point.x
    frame.transform.translation.y = pt.point.y
    frame.transform.translation.z = pt.point.z
    
    frame.transform.rotation.x = 0
    frame.transform.rotation.y = 0
    frame.transform.rotation.z = 0
    frame.transform.rotation.w = 1
    
    return frame


def transform_point(point, frame):
    """ Transforms a PointStamped message into a new frame """
    listener = tf.TransformListener()
    listener.waitForTransform(point.header.frame_id, frame, rospy.Time(0), rospy.Duration(3.0))
    point_transformed = listener.transformPoint(frame, point)
    return point_transformed


def find_switch(request):
    """ Do all the things for finding a switch with human help. """

    ########  Find Switches with Human Help #########
    grab_pt = Grabber('/head_mount_kinect/rgb/image_rect_color',
                      'Click on one or more switches')
    grab_pt.need_image = True
    while grab_pt.need_image:
        rospy.sleep(0.01)
    point_from_pixel = PointFromPixel('/head_mount_kinect/depth_registered/camera_info', 
                                      '/head_mount_kinect/depth_registered/image_rect')
    while point_from_pixel.need_camera_info or point_from_pixel.need_depth_image:
        rospy.sleep(0.01)
    bridge = CvBridge()
    grab_pt.image = bridge.imgmsg_to_cv( grab_pt.image )
    grab_pt.image = numpy.asarray( grab_pt.image )
    cv2.imshow(grab_pt.window, grab_pt.image)  # display image
    pt_switch = []  # list of 3D switch locations
    names_switch = []
    while grab_pt.need_point:
        cv2.imshow(grab_pt.window, grab_pt.image)  # display for review
        cv.SetMouseCallback(grab_pt.window, grab_pt.callback_point)  # get mouse input
        cv.WaitKey(0)
        if not grab_pt.need_point:  # if we got a point
            pt_switch.append(point_from_pixel.calculate_3d_point(grab_pt.point))  # get 3d point that corresponds to pixel
            names_switch.append(raw_input('Name this switch: '))
        grab_pt.need_point = not grab_pt.need_point  # if got a point, loop; else continue

    # Assemble response to service request
    response = FindSwitchResponse()
    for pt, name in zip(pt_switch, names_switch):
        frame = make_point_tf(request.device, transform_point(pt, request.device))
        response.frames.append( frame )
        response.names.append( name )
    return response


if __name__ == '__main__':

    rospy.init_node('find_switch')
    srv = rospy.Service('find_switch', FindSwitch, find_switch)
    srv.spin()
