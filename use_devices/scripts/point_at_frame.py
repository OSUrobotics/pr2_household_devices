#!/usr/bin/env python

# NOTE: much of this code comes straight from use_button.py

import roslib
roslib.load_manifest('use_devices')

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from actionlib import SimpleActionClient
from cartesian_motion.srv import *
from use_devices.srv import *


class Pointer():
    def __init__(self):
        rospy.wait_for_service('move_gripper')
        self.move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
        
    def point_at_frame(self, request):
        """ Moves gripper to align with a device or control. """
        # Unpack request variables
        which_arm = request.which_arm
        frame = request.frame
        standoff = request.standoff
        speed = request.speed

        offset_zeros = Vector3Stamped()
        offset_zeros.header.frame_id = frame

        to_gripper_tip = 0.18 + 0.036/2 + 0.018
        target = PoseStamped()
        target.header.frame_id = frame
        target.pose.position.z = to_gripper_tip + standoff
        target.pose.orientation.x =  0.5
        target.pose.orientation.y = -0.5
        target.pose.orientation.z = -0.5
        target.pose.orientation.w = -0.5
        
        request = MoveGripperRequest()
        request.which_arm = which_arm
        request.pose_goal = target
        request.speed = speed
        request.goal_offset = offset_zeros
        response_move = self.move_gripper(request)

        # Construct response
        response_point = PointAtFrameResponse()
        response_point.success = response_move.success
        return response_point

if __name__ == '__main__':
    rospy.init_node('point_at_frame')
    pointer = Pointer()
    srv = rospy.Service('point_at_frame', PointAtFrame, pointer.point_at_frame)
    rospy.spin()
