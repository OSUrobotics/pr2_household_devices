#!/usr/bin/env python

# NOTE: much of this code comes straight from use_button.py

import roslib
roslib.load_manifest('use_switch')

import rospy
import tf
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyRequest
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from actionlib import SimpleActionClient
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from cartesian_motion.srv import *
from use_switch.srv import *


def go_to_switch(which_arm, name, standoff, offset, speed):
    """ Moves gripper to align with switch. 
    INPUTS: name of switch frame, Z-direction offset, adjustment vector, speed, flip direction """
    to_gripper_tip = 0.18 + 0.036/2 + 0.018
    target = PoseStamped()
    target.header.frame_id = name
    target.pose.position.x = offset.vector.x
    target.pose.position.y = offset.vector.y
    target.pose.position.z = offset.vector.z + to_gripper_tip + standoff
    target.pose.orientation.x =  0.0
    target.pose.orientation.y =  0.707
    target.pose.orientation.z =  0.0
    target.pose.orientation.w =  0.707

    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    request = MoveGripperRequest()
    request.which_arm = which_arm
    request.pose_goal = target
    request.speed = speed
    request.goal_offset = offset
    response = move_gripper(request)
    return response


def shift_pose(which_arm, pose_current, axis, distance, speed):
    """ For moving the gripper small distances along one axis. 
    This is my tool for motion RELATIVE to current pose. """
    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    request = MoveGripperRequest()
    request.which_arm = which_arm
    request.pose_goal = pose_current
    coord = pose_current.pose.position.__getattribute__(axis)
    coord += distance  # shift current coordinate by specified amount
    request.pose_goal.pose.position.__setattr__(axis, coord) 
    request.speed = speed
    offset_zeros = Vector3Stamped()
    offset_zeros.header.frame_id = pose_current.header.frame_id
    request.goal_offset = offset_zeros
    response_shift = move_gripper(request)
    return response_shift


def open_gripper(which_arm):
    """ Open the gripper """
    gripper_client = SimpleActionClient(which_arm[0] + '_gripper_controller/gripper_action', 
                                        Pr2GripperCommandAction)
    gripper_client.wait_for_server()

    ##### Define goals for gripper: open all the way
    goal_open = Pr2GripperCommandGoal()
    goal_open.command.position = 0.05
    goal_open.command.max_effort = 20.0
    
    gripper_client.send_goal_and_wait(goal_open)  # open gripper for switch-flipping


def use_switch(request):
    """ Move to standoff, open gripper, close in, then flip switch """
    # Unpack request message
    which_arm = request.which_arm
    name = request.switch
    offset = request.goal_offset
    standoff = request.standoff
    up_or_down = request.up_or_down

    lis = tf.TransformListener()

    # Attempt to flip switch; construct response 
    response_use_switch = UseSwitchResponse()
    response_approach = go_to_switch(which_arm, name, standoff, offset, 0.20)

    open_gripper(request.which_arm)  # open gripper to flip better

    # Re-calibrate pressure sensors
    calibrate_pressure = rospy.ServiceProxy('calibrate_pressure', Empty_srv)
    request = EmptyRequest()
    calibrate_pressure(request)

    if response_approach.success:
        response_engage = go_to_switch(which_arm, name, -0.10, offset, 0.04)  # touch switch panel
        response_recoil = shift_pose(which_arm, response_engage.final_pose, 'z', 0.003, 0.02)  # pull back a bit

        # Attempt the flip
        if up_or_down == 'up':
            distance_flip =  0.06
        elif up_or_down == 'down':
            distance_flip = -0.06
        else:
            rospy.loginfo('Specify either "up" or "down" for flipping!')
            return UseSwitchResponse()
        response_flip = shift_pose(which_arm, response_recoil.final_pose, 'y', distance_flip, 0.02)
        
        go_to_switch(which_arm, name, standoff, offset, 0.20)
        if response_flip.made_contact:
            response_use_switch.success = True
        else:
            response_use_switch.success = False
    else:
        response_use_switch.success = False
    return response_use_switch


if __name__ == '__main__':

    rospy.init_node('use_switch')
    srv = rospy.Service('use_switch', UseSwitch, use_switch)
    rospy.spin()

