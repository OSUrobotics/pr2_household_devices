#!/usr/bin/env python

import roslib
roslib.load_manifest('device_interface')

import rospy
import tf
import numpy
from math import sin, cos, atan2
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from device_interface.srv import *


class BroadcastTFs():
    """ Broadcasts multiple TF frames. Each frame is added by a service call. """
    def __init__(self):
        self.listener = tf.TransformListener()
        self.srv1 = rospy.Service('send_tf', SendTF, self.get_tf)
        self.srv2 = rospy.Service('level_tf', LevelTF, self.level_tf)
        self.frame_names = []
        self.transforms = []
        self.broadcast_transforms()  # stand by to broadcast transforms once received


    def get_tf(self, request):
        frame = []
        frame.append((request.frame.transform.translation.x,
                      request.frame.transform.translation.y,
                      request.frame.transform.translation.z))  # add translation tuple
        frame.append((request.frame.transform.rotation.x,
                      request.frame.transform.rotation.y,
                      request.frame.transform.rotation.z,
                      request.frame.transform.rotation.w))  # add rotation tuple
        frame.append(request.frame.header.frame_id)
        self.transforms.append(frame)
        self.frame_names.append(request.frame_name)
        rospy.loginfo('Got a transform!')
        return True


    def level_tf(self, request):
        """ Given an existing frame, rotates it about the z-axis so the +y-axis is aligned with the +z-axis of its parent frame. """
        # Unpack request variables
        frame_child = request.frame_child
        frame_parent = request.frame_parent

        # Get transform from parent to child
        self.listener.waitForTransform(frame_parent, frame_child, rospy.Time(0), rospy.Duration(3.0))
        transform = self.listener.lookupTransform(frame_parent, frame_child, rospy.Time(0))
        transform = [ transform[0], transform[1], frame_parent ]
        q_old = numpy.array( transform[1] ) 

        # Construct vertical vector in static (parent) frame
        vert = Vector3Stamped()
        vert.header.frame_id = frame_parent
        vert.vector.z = 1.0

        # Transform vertical vector into target (child) frame
        self.listener.waitForTransform(frame_child, frame_parent, rospy.Time(0), rospy.Duration(3.0))
        vert_tf = self.listener.transformVector3(frame_child, vert)
        vert_tf.vector.z = 0.0  # project into XY plane

        # Get quaternion rotation that will rotate child frame as desired
        v = numpy.array([vert_tf.vector.__getattribute__(key) for key in 'xyz'])  # world vertical axis
        j = numpy.array([0., 1., 0.])  # current device y-axis
        q_vert = self.quaternion_from_directions(v, j)

        # Multiply new quaternion into existing one
        q_new = self.multiply_quaternions(q_old, q_vert)

        # Apply new rotation to frame
        transform[1] = tuple( q_new )
        index = self.frame_names.index( frame_child )  # find index of our target frame
        self.transforms[ index ] = transform  # Update frame in broadcast list
        
        # Construct response
        response = LevelTFResponse()
        response.success = True
        return response


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

        
    def multiply_quaternions(self, q1, q2):
        """ Multiply two quaternions. In terms of rotations, does first rotation followed by second one.
        INPUTS: two four-element numpy vectors of form [x, y, z, w] """
        s = q1[3]; v = q1[:3]
        t = q2[3]; w = q2[:3]
        # Equation: q1*q2 = (s+v_bar)*(t+w_bar) = (s*t - v DOT w) + (s*w + t*v + v CROSS w)
        q_vector = s*w + t*v + numpy.cross(v, w)
        q_scalar = s*t - numpy.dot(v, w)
        q_product = numpy.append(q_vector, q_scalar)
        return q_product


    def broadcast_transforms(self):
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            if self.transforms:
                for frame, frame_name in zip(self.transforms, self.frame_names):
                    br.sendTransform(frame[0], 
                                     frame[1], 
                                     rospy.Time.now(),
                                     frame_name,
                                     frame[2])
            rospy.Rate(10)


if __name__ == '__main__':
    rospy.init_node('broadcast_TFs')
    broadcaster = BroadcastTFs()

