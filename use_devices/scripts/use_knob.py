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
from std_msgs.msg import String
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyRequest
from actionlib import SimpleActionClient, GoalStatus
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_gripper_sensor_msgs.msg import PR2GripperForceServoAction, PR2GripperForceServoGoal
from pr2_gripper_sensor_msgs.msg import PR2GripperFindContactAction, PR2GripperFindContactGoal
from PySide.QtCore import *
from PySide.QtGui import *
from math import pi
from use_knob.srv import *
from cartesian_motion.srv import *


class TurnKnobGUI():
    def __init__(self, argv, which_arm, topic):
        """ Set up dial GUI """
	self.which_arm = which_arm

        # Ready the turner!
        rospy.wait_for_service('turn_gripper')
        self.turn_gripper = rospy.ServiceProxy('turn_gripper', TurnGripper)
        self.value_current = 0.0

        # GUI Stuff
        self.app = QApplication(argv)
        self.group = QGroupBox('Turn Dial and Press Go!')
        self.format_layout()
        self.group.setLayout(self.format_layout())

        self.displayer = DisplayKnob(topic)

    def format_layout(self):
        """ Make GUI dial, buttons, etc. """
        layout = QVBoxLayout()
        dial = QDial()
        dial.valueChanged.connect(self.callback)
        dial.setNotchesVisible(True)
        dial.setWrapping(True)
        layout.addWidget(dial)

        button_go = QPushButton('Go!')
        button_go.clicked.connect(self.turn_wrist)
        layout.addWidget(button_go)

        button_show = QPushButton('Show Knob')
        button_show.clicked.connect(self.show)
        layout.addWidget(button_show)

        button_done = QPushButton('Done')
        button_done.clicked.connect(self.done)
        layout.addWidget(button_done)

        return layout

    def callback(self, value):
        """ Save dial value whenever it changes """
        self.value_abs = float(value) / 99.0 * 360.0
        self.value_rel = self.value_abs - self.value_current
        if self.value_rel > 180.0:
            self.value_rel -= 360.0
        #rospy.loginfo('Dial value ABSOLUTE: {0}'.format(self.value_abs))
        #rospy.loginfo('Dial value RELATIVE: {0}'.format(self.value_rel))

    def turn_wrist(self):
        """ For turning the gripper RELATIVE to current angle. """
        request = TurnGripperRequest()
        request.which_arm = self.which_arm
        request.degrees_CW = self.value_rel
        response = self.turn_gripper(request)
        self.value_current = self.value_abs

    def show(self):
        """ Show image of knob """
        self.displayer.request_image()

    def done(self):
        """ Close GUI when user presses "Done" button """
        self.group.hide()
        self.app.quit()

    def execute(self):
        """ Launch GUI """
        self.show()  # show initial knob position
        self.group.show()
        self.app.exec_()
        return 'succeeded'

class DisplayKnob():
    """ Uses forearm camera to show the current state of the knob """
    def __init__(self, topic):
        self.want_image = False
        rospy.Subscriber(topic, Image, self.callback_image, queue_size=1)

    def callback_image(self, image):
        """ If currently want an image, this gets it """
        if self.want_image:
            self.display_image(image)
            rospy.loginfo('Got image!')
            self.want_image = False

    def request_image(self):
        """ Wait until get an image, then display it """
        self.want_image = True
        while self.want_image:
            rospy.sleep(0.01)

    def display_image(self, image):
        """ Displays image using OpenCV """
        bridge = CvBridge()
        image_cv = bridge.imgmsg_to_cv(image)
        window = 'Current State of Knob'
        cv.ShowImage(window, image_cv)  # display image
        rospy.sleep(1.0)




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
        self.balance = balance

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

def go_to_knob(which_arm, name, standoff, offset, speed):
    """ Moves gripper to align with knob.
    INPUTS: name of knob frame, Z-direction offset, adjustment vector, speed """
    to_gripper_tip = 0.18 + 0.036/2 + 0.018
    target = PoseStamped()
    target.header.frame_id = name
    target.pose.position.x = offset.vector.x
    target.pose.position.y = offset.vector.y
    target.pose.position.z = offset.vector.z + to_gripper_tip + standoff
    target.pose.orientation.x = 0.0
    target.pose.orientation.y = 0.707
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.707

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

def use_knob(request):
    """ Move to standoff, then use knob """
    # Unpack request message
    which_arm = request.which_arm
    name = request.knob
    offset = request.goal_offset
    standoff = request.standoff

    listener = tf.TransformListener()

    # Get close to knob
    go_to_knob(which_arm, name, standoff + 0.05, offset, 0.20)  # align with knob
    gripper = Gripper()

    # Open gripper & re-calibrate pressure sensors
    gripper.open()
    calibrate_pressure = rospy.ServiceProxy('calibrate_pressure', Empty_srv)
    request = EmptyRequest()
    calibrate_pressure(request)

    # Engage with knob
    response = go_to_knob(which_arm, name, standoff, offset, 0.20)   # move to standoff quickly
    response = go_to_knob(which_arm, name, -0.10, offset, 0.05)   # move towards knob slowly until sense contact
    response = shift_pose(which_arm, response.final_pose, 'z', 0.02, 0.02)  # Pull back a bit

    # Grab knob -- with feeling!
    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    centered = False
    shift_distance = 0.02
    while not centered:
        gripper.findTwoContacts()  # grip the knob lightly
        balance = gripper.balance.data  # get the grip balance
        rospy.loginfo(balance)
        gripper.open()

        if balance == 'CENTERED':
            centered = True
        elif balance == 'NO_CONTACT':
            rospy.logwarn('WARNING: No contact detected -- gripper may not have closed completely!')
            calibrate_pressure(request)
        else:
            # Construct vector by which to shift the gripper
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
          
            # Construct current pose of right wrist
            wrist_pose = PoseStamped()  
            wrist_pose.header.frame_id = '/r_wrist_roll_link'
            wrist_pose.pose.orientation.w = 1.0
            
            # Command gripper to offset its pose by the shift vector
            request = MoveGripperRequest()
            request.which_arm = 'right'
            request.pose_goal = wrist_pose
            request.speed = 0.01
            request.goal_offset = shift
            response = move_gripper(request)

            shift_distance *= 0.50  # reduce shift distance to home in on a good grasp

            # After several tries, stop adjusting and try a grasp
            if shift_distance < 0.002:
                rospy.logwarn('WARNING: could not find a nice center for grasping -- may be suboptimal!')
                break

    # Grab knob
    gripper.findTwoContacts()
    gripper.hold(1.0)

    # Launch GUI for turning wrist
    turn_knob = TurnKnobGUI(sys.argv,
                            which_arm, 
                            which_arm[0] + '_forearm_cam/image_rect_color')
    turn_knob.execute()
    del turn_knob
    
    # Finish up
    gripper.open()
    response = go_to_knob(which_arm, name, standoff, offset, 0.20)  # pull away from knob
    response = UseKnobResponse()
    response.success = True  # HACK
    return response


if __name__ == '__main__':

    rospy.init_node('use_knob')
    srv1 = rospy.Service('use_knob', UseKnob, use_knob)
    srv2 = rospy.Service('turn_knob', TurnKnob, turn_knob)
    rospy.spin()



"""
def turn_knob(request):
    # Turn knob a specified amount (for performing a pre-programmed task). 
    # Unpack request message
    which_arm = request.which_arm
    name = request.knob
    offset = request.goal_offset
    standoff = request.standoff
    degrees_CW = request.degrees_CW

    response_turn_knob = TurnKnobResponse()

    # Open gripper & engage with knob
    gripper = Gripper()
    gripper.open()
    response_engage = go_to_knob(which_arm, name, -0.10, offset, 0.05)
    
    # Pull back a bit
    response_shift = shift_pose(which_arm, response_engage.final_pose, 'z', 0.008, 0.02)
    
    # Grab knob
    gripper.findTwoContacts()
    gripper.hold( 5.0 )
    
    # Turn the wrist!
    rospy.wait_for_service('turn_gripper')
    turn_gripper = rospy.ServiceProxy('turn_gripper', TurnGripper)
    request = TurnGripperRequest()
    request.which_arm = which_arm
    request.degrees_CW = degrees_CW
    response = turn_gripper(request)
    rospy.sleep(1.0)
    
    # Release knob and return to standoff; close gripper
    gripper.open()
    go_to_knob(which_arm, name, standoff, offset, 0.05)
    gripper.close()

    response_turn_knob.success = True

    return response_turn_knob
"""
