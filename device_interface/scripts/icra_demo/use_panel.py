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
    devices = {'switch_panel': {'knob': [], 'switch': ['light_switch'], 'button': [], 'keypad': []}}
    device_name = 'switch_panel'


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


    #############  Use Stuff  ##################

    # Ready the adjuster
    rospy.wait_for_service('adjust_gripper_pose')
    adjust_gripper_pose = rospy.ServiceProxy('adjust_gripper_pose', AdjustGripperPose)

    # Ready the pointer
    rospy.wait_for_service('point_at_frame')
    point_at_frame = rospy.ServiceProxy('point_at_frame', PointAtFrame)

    for control_to_use in ['light_switch']:
        #control_to_use = raw_input('Name of control to use: ')

        # Point at control
        request = PointAtFrameRequest()
        request.which_arm = 'right'
        request.frame = control_to_use
        request.standoff = 0.05
        request.speed = 0.20
        response = point_at_frame(request)

        # Get gripper offset
        request = AdjustGripperPoseRequest()
        request.frame = control_to_use
        response = adjust_gripper_pose(request)
        if response.success:
            offset = response.adjust
        else:
            offset = offset_zeros  # do not offset if adjustment failed
        rospy.loginfo('Offset for using "{0}" is: {1}'.format(control_to_use, offset.vector))

                
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

    request = MoveGripperRequest()
    request.which_arm = 'right'
    request.stored_goal = 'READY'
    request.speed = 0.10  # m/s
    offset_zeros = Vector3Stamped()
    offset_zeros.header.frame_id = '/base_link'
    request.goal_offset = offset_zeros  # no offset
    response = move_gripper(request)
