#!/usr/bin/env python

import roslib
roslib.load_manifest('use_button')

import rospy
import tf
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyRequest
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from actionlib import SimpleActionClient
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from cartesian_motion.srv import *
from use_button.srv import *


def go_to_button(which_arm, name, standoff, offset, speed):
    """ Moves gripper to align with button. 
    INPUTS: name of button frame, Z-direction offset, adjustment vector, speed """
    to_gripper_tip = 0.18 + 0.036/2 + 0.018
    target = PoseStamped()
    target.header.frame_id = name
    target.pose.position.x = offset.vector.x
    target.pose.position.y = offset.vector.y
    target.pose.position.z = offset.vector.z + to_gripper_tip + standoff
    target.pose.orientation.x = 0.5
    target.pose.orientation.y = -0.5
    target.pose.orientation.z = -0.5
    target.pose.orientation.w = -0.5

    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    request = MoveGripperRequest()
    request.which_arm = which_arm
    request.pose_goal = target
    request.speed = speed
    request.goal_offset = offset
    response = move_gripper(request)
    return response

def close_gripper(which_arm):
    """ close the gripper """
    gripper_client = SimpleActionClient(which_arm[0] + '_gripper_controller/gripper_action', Pr2GripperCommandAction)
    gripper_client.wait_for_server()

    ##### Define goals for gripper: close all the way
    goal_close = Pr2GripperCommandGoal()
    goal_close.command.position = 0.0   
    goal_close.command.max_effort = 20.0
    
    gripper_client.send_goal_and_wait(goal_close)  # open gripper for knob

def use_button(request):
    """ Move to standoff, adjust gripper pose, then push button """
    # Unpack request message
    which_arm = request.which_arm
    name = request.button
    offset = request.goal_offset
    standoff = request.standoff

    close_gripper(request.which_arm)

    # Re-calibrate pressure sensors
    calibrate_pressure = rospy.ServiceProxy('calibrate_pressure', Empty_srv)
    request = EmptyRequest()
    calibrate_pressure(request)

    # Attempt to push button; construct response 
    response_use_button = UseButtonResponse()
    response_approach = go_to_button(which_arm, name, standoff, offset, 0.20)
    if response_approach.success:
        response_push = go_to_button(which_arm, name, -0.10, offset, 0.04)
        go_to_button(which_arm, name, standoff, offset, 0.20)
        if response_push.made_contact:
            response_use_button.success = True
        else:
            response_use_button.success = False
    else:
        response_use_button.success = False
    return response_use_button


if __name__ == '__main__':

    rospy.init_node('use_button')
    srv = rospy.Service('use_button', UseButton, use_button)
    rospy.spin()
