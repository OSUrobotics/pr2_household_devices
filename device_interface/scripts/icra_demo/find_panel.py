#!/usr/bin/env python

import roslib
roslib.load_manifest('device_interface')

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from cartesian_motion.srv import *
from gripper_position_feedback.srv import *
from use_devices.srv import *
from use_knob.srv import *
from use_button.srv import *
from use_switch.srv import *
from device_interface.srv import *


if __name__ == '__main__':

    rospy.init_node('device_interface')
    devices = {}

    #########  Get Ready  ############

    # Move to READY position
    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    request = MoveGripperRequest()
    request.which_arm = 'right'
    request.stored_goal = 'READY'
    request.speed = 0.10  # m/s
    offset_zeros = Vector3Stamped()
    offset_zeros.header.frame_id = '/base_link'
    request.goal_offset = offset_zeros  # no offset
    response = move_gripper(request)


    #########  Find Stuff  #################

    # Find a device
    rospy.wait_for_service('find_device')
    find_device = rospy.ServiceProxy('find_device', FindDevice)
    request = FindDeviceRequest()
    response = find_device(request)
    frame = response.device
    plane = response.plane_coefficients
    device_name = 'switch_panel'
    devices[device_name] = {'knob': [], 'switch': [], 'button': [], 'keypad': []}

    # Broadcast device frame
    rospy.wait_for_service('send_tf')
    send_tf = rospy.ServiceProxy('send_tf', SendTF)
    request = SendTFRequest()
    request.frame = frame
    request.frame_name = device_name
    response = send_tf(request)

    # Align device frame with the ground
    rospy.wait_for_service('level_tf')
    level_tf = rospy.ServiceProxy('level_tf', LevelTF)
    request = LevelTFRequest()
    request.frame_child = device_name
    request.frame_parent = '/base_link'  # new parent frame
    response = level_tf(request)

    # Find some switches
    rospy.wait_for_service('find_switch')
    find_switch = rospy.ServiceProxy('find_switch', FindSwitch)
    request = FindSwitchRequest()
    request.device = device_name
    response = find_switch(request)
    frames = response.frames
    names = ['light_switch']

    # Broadcast frames of switches
    for frame, name in zip(frames, names):
        request = SendTFRequest()
        request.frame = frame
        request.frame_name = name
        response = send_tf(request)
        devices[device_name]['switch'].append(name)

    # Summarize
    rospy.loginfo('Here\'s what we have found: {0}'.format(devices))
