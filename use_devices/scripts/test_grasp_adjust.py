#!/usr/bin/env python

import roslib
roslib.load_manifest('use_knob')

import rospy
import sys
import tf
import cv
import cv2
import numpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import Image
from actionlib import SimpleActionClient, GoalStatus
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_gripper_sensor_msgs.msg import PR2GripperForceServoAction, PR2GripperForceServoGoal
from pr2_gripper_sensor_msgs.msg import PR2GripperFindContactAction, PR2GripperFindContactGoal
from PySide.QtCore import *
from PySide.QtGui import *
from math import pi
from use_knob.srv import *
from cartesian_motion.srv import *
from std_msgs.msg import String



class Gripper():
    """ Wrapper for all the action client stuff """
    def __init__(self):
        """ Instantiate action clients and wait for communication with servers """
        self.client_gripper = SimpleActionClient("r_gripper_sensor_controller/gripper_action", Pr2GripperCommandAction)
        self.client_contact = SimpleActionClient("r_gripper_sensor_controller/find_contact", PR2GripperFindContactAction)
        self.client_force = SimpleActionClient("r_gripper_sensor_controller/force_servo", PR2GripperForceServoAction)
        
        self.client_gripper.wait_for_server(rospy.Duration(10.0))
        self.client_contact.wait_for_server(rospy.Duration(10.0))
        self.client_force.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo('All servers ready!')

        rospy.Subscriber('r_gripper_pressure_balance', String, self.store_balance)

    def store_balance(self, balance):
        """ Stores gripper balance. """
        self.balance = balance.data

    def open(self):
        """ Open the gripper """
        goal_open = Pr2GripperCommandGoal()
        goal_open.command.position = 0.09
        goal_open.command.max_effort = -1.0
        
        rospy.loginfo("Sending goal: OPEN")
        self.client_gripper.send_goal_and_wait(goal_open)
        if (self.client_gripper.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('SUCCESS. The gripper opened!')
        else:
            rospy.loginfo('FAILURE. The gripper failed to open.')

    def open_manual(self, position):
        """ Open the gripper """
        goal_open = Pr2GripperCommandGoal()
        goal_open.command.position = position
        goal_open.command.max_effort = -1.0
        
        rospy.loginfo("Sending goal: OPEN")
        self.client_gripper.send_goal_and_wait(goal_open)
        if (self.client_gripper.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('SUCCESS. The gripper opened!')
        else:
            rospy.loginfo('FAILURE. The gripper failed to open.')

    def close(self):
        """ Close the gripper """
        goal_close = Pr2GripperCommandGoal()
        goal_close.command.position = 0.00
        goal_close.command.max_effort = 20.0
        
        rospy.loginfo("Sending goal: CLOSE")
        self.client_gripper.send_goal_and_wait(goal_close)
        if (self.client_gripper.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('SUCCESS. The gripper closed!')
        else:
            rospy.loginfo('FAILURE. The gripper failed to close.')

    def hold(self, holdForce):
        """ Hold something with a specified grip force """
        goal_squeeze = PR2GripperForceServoGoal()
        goal_squeeze.command.fingertip_force = holdForce
        
        rospy.loginfo("Sending goal: HOLD with {0}N of force.".format(holdForce))
        self.client_force.send_goal_and_wait( goal_squeeze )
        if (self.client_force.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('SUCCESS. Stable force achieved!')
        else:
            rospy.loginfo('FAILURE. Stable force was NOT achieved.')
        
    def findTwoContacts(self):
        """ Find two contacts and go into force control mode """
        goal_contact = PR2GripperFindContactGoal()
        goal_contact.command.contact_conditions = goal_contact.command.BOTH  # close until both fingers contact
        goal_contact.command.zero_fingertip_sensors = True  # zero fingertip sensors before moving

        rospy.loginfo("Sending goal: FIND TWO CONTACTS")
        self.client_contact.send_goal_and_wait(goal_contact)
        if (self.client_contact.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('SUCCESS. Contact found at left: {0} and right: {1}'.format(self.client_contact.get_result().data.left_fingertip_pad_contact, 
                                                                                      self.client_contact.get_result().data.right_fingertip_pad_contact))
            rospy.loginfo('Contact force for left: {0} and right: {1}'.format(self.client_contact.get_result().data.left_fingertip_pad_force, 
                                                                              self.client_contact.get_result().data.right_fingertip_pad_force))
        else:
            rospy.loginfo('FAILURE. No contact found or force not able to be maintained')


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


def test_grasp_adjust():
    """ Test ability to adjust a grasp so as not to 'SNOW CONE' the object. 
    SNOW CONE: In baseball, a catch made by a fielder or basemen where
    part of the baseball is sticking out of the top webbing of the
    glove resembling a snow cone."""

    which_arm = 'right'
    listener = tf.TransformListener()
    rospy.sleep(6.0)


    # Open gripper & engage with object
    gripper = Gripper()
    gripper.open()

    # Initialize mover
    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)

    
    # Feel it out
    centered = False
    shift_distance = 0.02
    while not centered:
        gripper.findTwoContacts()
        balance = gripper.balance  # take a snapshot of the balance status
        rospy.loginfo(balance)
        gripper.open()

        if balance == 'CENTERED':
            centered = True
        elif balance == 'NO_CONTACT':
            rospy.logwarn('WARNING: No contact detected -- gripper may not have closed completely!')
            continue
        else:
            shift = Vector3Stamped()
            shift.header.frame_id = 'r_gripper_l_finger_tip_frame'
            frame = '/r_wrist_roll_link'
            shift.header.stamp = rospy.Time(0)
            if balance == 'GO_POSITIVE':
                shift.vector.z = shift_distance 
            elif balance == 'GO_NEGATIVE':
                shift.vector.z = shift_distance * -1
            listener.waitForTransform(frame, shift.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            shift = listener.transformVector3(frame, shift)  # Transform into wrist frame
          
            wrist_pose = PoseStamped()  # current pose of right wrist
            wrist_pose.header.frame_id = '/r_wrist_roll_link'
            wrist_pose.pose.orientation.w = 1.0
            
            request = MoveGripperRequest()
            request.which_arm = 'right'
            request.pose_goal = wrist_pose
            request.speed = 0.01
            request.goal_offset = shift
            response = move_gripper(request)

            shift_distance *= 0.50  # reduce shift distance to home in on a good grasp

            if shift_distance < 0.002:
                rospy.logwarn('WARNING: could not find a nice center for grasping -- may be suboptimal!')
                break



    #gripper.open()
    


    # Grab knob
    gripper.findTwoContacts()
    gripper.hold(1.0)
    rospy.sleep(2.0)
    gripper.open()

    # Release knob and return to standoff; close gripper
    #gripper.open()
    #gripper.close()



if __name__ == '__main__':

    rospy.init_node('test_grasp_adjust')
    response = test_grasp_adjust()
