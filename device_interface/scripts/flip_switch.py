#!/usr/bin/env python

import roslib
roslib.load_manifest('device_interface')

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib 
from cartesian_motion.srv import *
#from gripper_position_feedback.srv import *
from find_devices.srv import *
from use_devices.srv import *
from device_interface.srv import *

#import rosservice

"""
    services = rosservice.get_service_list()
    print '1   ', services
    services = rosservice.rosservice_find('use_knob/UseKnob')
    print '2   ', services
    arguments = rosservice.get_service_args('/use_knob')
    print '3   ', arguments
"""


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

    # Move head to hard-coded position
    client = actionlib.SimpleActionClient(
        '/head_traj_controller/point_head_action', PointHeadAction)
    client.wait_for_server()
    
    g = PointHeadGoal()
    g.target.header.frame_id = 'base_link'
    g.target.point.x =  1.0
    g.target.point.y = -0.45
    g.target.point.z =  0.6
    g.min_duration = rospy.Duration(1.0)
    
    client.send_goal(g)
    client.wait_for_result()
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        print "Succeeded"
    else:
        print "Failed"



    #########  Find Stuff  #################

    # Find a device
    rospy.wait_for_service('find_device')
    find_device = rospy.ServiceProxy('find_device', FindDevice)
    request = FindDeviceRequest()
    response = find_device(request)
    frame = response.device
    plane = response.plane_coefficients
    device_name = raw_input('Name this device: ')
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
    """
    # Find a knob
    rospy.wait_for_service('find_knob')
    find_knob = rospy.ServiceProxy('find_knob', FindKnob)
    request = FindKnobRequest()
    request.device = device_name
    request.plane_coefficients = plane
    response = find_knob(request)
    frame = response.frame
    knob_height = response.height
    knob_name = raw_input('Name this knob: ')
    devices[device_name]['knob'].append(knob_name)

    # Broadcast knob frame
    request = SendTFRequest()
    request.frame = frame
    request.frame_name = knob_name
    response = send_tf(request)
    """
    # Find some switches
    rospy.wait_for_service('find_switch')
    find_switch = rospy.ServiceProxy('find_switch', FindSwitch)
    request = FindSwitchRequest()
    request.device = device_name
    response = find_switch(request)
    frames = response.frames
    names = response.names

    # Broadcast frames of switches
    for frame, name in zip(frames, names):
        request = SendTFRequest()
        request.frame = frame
        request.frame_name = name
        response = send_tf(request)
        devices[device_name]['switch'].append(name)
    """
    # Find some buttons
    rospy.wait_for_service('find_button')
    find_button = rospy.ServiceProxy('find_button', FindButton)
    request = FindButtonRequest()
    request.device = device_name
    response = find_button(request)
    frames = response.frames
    names = response.names

    # Broadcast frames of buttons
    for frame, name in zip(frames, names):
        request = SendTFRequest()
        request.frame = frame
        request.frame_name = name
        response = send_tf(request)
        devices[device_name]['button'].append(name)
    """    
    rospy.loginfo('Here\'s what we have found: {0}'.format(devices))


    #############  Use Stuff  ##################

    # Ready the adjuster
    #rospy.wait_for_service('adjust_gripper_pose')
    #adjust_gripper_pose = rospy.ServiceProxy('adjust_gripper_pose', AdjustGripperPose)

    # Ready the pointer
    rospy.wait_for_service('point_at_frame')
    point_at_frame = rospy.ServiceProxy('point_at_frame', PointAtFrame)

    while not rospy.is_shutdown():
        control_to_use = raw_input('Name of control to use: ')

        # Point at control
        request = PointAtFrameRequest()
        request.which_arm = 'right'
        request.frame = control_to_use
        request.standoff = 0.05
        request.speed = 0.20
        response = point_at_frame(request)

        # Get gripper offset
        """
        request = AdjustGripperPoseRequest()
        request.frame = control_to_use
        response = adjust_gripper_pose(request)
        if response.success:
            offset = response.adjust
        else:
            offset = offset_zeros  # do not offset if adjustment failed
        rospy.loginfo('Offset for using "{0}" is: {1}'.format(control_to_use, offset.vector))
        """
        offset = offset_zeros
                
        ###########  Call a service based on control type  #############

        # KNOB
        if control_to_use in devices[device_name]['knob']:
            rospy.wait_for_service('use_knob')
            use_knob = rospy.ServiceProxy('use_knob', UseKnob)
            request = UseKnobRequest()
            request.which_arm = 'right'
            request.knob = control_to_use
            request.goal_offset = offset
            request.standoff = 0.06 + knob_height
            response = use_knob(request)
            
        # SWITCH
        if control_to_use in devices[device_name]['switch']:
            rospy.wait_for_service('use_switch')
            use_switch = rospy.ServiceProxy('use_switch', UseSwitch)
            request = UseSwitchRequest()
            request.which_arm = 'right'
            request.switch = control_to_use
            request.goal_offset = offset
            request.standoff = 0.05
            request.up_or_down = 'up'
            response = use_switch(request)

        # BUTTON
        if control_to_use in devices[device_name]['button']:
            rospy.wait_for_service('use_button')
            use_button = rospy.ServiceProxy('use_button', UseButton)
            request = UseButtonRequest()
            request.which_arm = 'right'
            request.button = control_to_use
            request.goal_offset = offset
            request.standoff = 0.05
            response = use_button(request)
