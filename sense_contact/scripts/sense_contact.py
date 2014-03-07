#!/usr/bin/env python

import roslib
roslib.load_manifest('sense_contact')

import rospy
from actionlib import SimpleActionClient, GoalStatus
from pr2_msgs.msg import PressureState
from std_msgs.msg import String
from std_msgs.msg import Empty as Empty_msg
import copy
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse


class PublishCalibratedPressures():
    """ Wrapper for all the action client stuff """
    def __init__(self, calibration, threshold_poke, threshold_grab):
        """ Instantiate action clients and wait for communication with servers """
        self.calibration = calibration
        self.threshold_poke = threshold_poke
        self.threshold_grab = threshold_grab
        rospy.loginfo('Poking threshold is {0}.'.format(self.threshold_poke))
        rospy.loginfo('Grabbing threshold is {0}.'.format(self.threshold_grab))
        self.is_in_contact = False
        self.sub = rospy.Subscriber('/pressure/r_gripper_motor', 
                                    PressureState, 
                                    self.detect_contact, 
                                    queue_size=1)
        self.pub_contact = rospy.Publisher('r_gripper_contact', Empty_msg)
        self.pub_pressures = rospy.Publisher('r_gripper_pressure_calibrated', PressureState)
        self.pub_balance = rospy.Publisher('r_gripper_pressure_balance', String)
        self.msg = Empty_msg()
        rospy.loginfo('Detecting contact for right arm.')
        
        while not rospy.is_shutdown():
            if self.is_in_contact:
                rospy.loginfo('PUBLISHER> Disabling contact detector for 3.0s ...')
                rospy.sleep(3.0)
                self.is_in_contact = False
            rospy.sleep(0.1)

    def detect_contact(self, pressure):
        """ Detect contact as finger side impact """
        pressure = self.zero_pressure(pressure)
        self.pub_pressures.publish(pressure)
        self.pressure_imbalance(pressure)
        pressure_tips = self.parse_pressure(pressure)
        is_in_contact = [p > self.threshold_poke for p in pressure_tips]
        if any(is_in_contact) and not self.is_in_contact:
            self.pub_contact.publish(self.msg)
            rospy.loginfo('PUBLISHER> Contact detected!')
            self.is_in_contact = True

    def pressure_imbalance(self, pressure):
        """ Detects whether gripper is holding something off-center. """
        readings = pressure.l_finger_tip  # arbitrarily choose one finger
        cells = {}
        side_readings = {}
        contact = {}
        cells['left']  = [7, 10, 13, 16, 19]  # indices for pressure cells
        cells['right'] = [9, 12, 15, 18, 21]
        for side, indices in cells.iteritems():
            side_readings[side] = [readings[i] > self.threshold_grab for i in indices]
            contact[side] = any(side_readings[side])
        if contact['left'] and contact['right']:
            balance = 'CENTERED'
        elif contact['left'] and not contact['right']:
            balance = 'GO_POSITIVE'
        elif contact['right'] and not contact['left']:
            balance = 'GO_NEGATIVE'
        else:
            balance = 'NO_CONTACT'
        self.pub_balance.publish(balance)
    
    def parse_pressure(self, pressure):
        """ Grabs just the pressures we want, returns a list. """
        return [pressure.l_finger_tip[3], 
                pressure.l_finger_tip[4],
                pressure.r_finger_tip[3],
                pressure.r_finger_tip[4]]
    
    def zero_pressure(self, pressure):
        """ Subtracts calibration from pressure reading. """
        pressure.l_finger_tip = [p - c for p, c in zip(pressure.l_finger_tip, self.calibration.l_finger_tip)]
        pressure.r_finger_tip = [p - c for p, c in zip(pressure.r_finger_tip, self.calibration.r_finger_tip)]
        return pressure


class CalibratePressure():
    """ Collect pressure readings for a short time against which to compare. """
    def __init__(self, how_many):
        self.counter_max = how_many
        self.sub = rospy.Subscriber('/pressure/r_gripper_motor', 
                                    PressureState, 
                                    self.gather_readings)
        self.calibrate_pressure(None)  # calibrate once to initialize

    def calibrate_pressure(self, request):
        """ Response to service request. """
        rospy.loginfo('Collecting stuff for calibration...')
        self.readings = []
        self.counter = self.counter_max
        while self.counter > 0:
            rospy.sleep(0.01)
        self.calibration = self.average_pressures(self.readings)
        rospy.loginfo('Calibration is: {0}'.format(self.calibration))
        rospy.loginfo('Done calibrating!')
        return EmptyResponse()

    def gather_readings(self, pressure):
        """ Save pressure readings. """
        if self.counter > 0:
            self.readings.append(copy.deepcopy(pressure))
            self.counter -= 1

    def average_pressures(self, pressures):
        """ Averages all the pressures. """
        average = copy.deepcopy(pressures[0])
        for pressure in pressures[1:]:
            average.l_finger_tip = [a + p for a, p in zip(average.l_finger_tip, pressure.l_finger_tip)]
            average.r_finger_tip = [a + p for a, p in zip(average.r_finger_tip, pressure.r_finger_tip)]
        average.l_finger_tip = [p / len(pressures) for p in average.l_finger_tip]
        average.r_finger_tip = [p / len(pressures) for p in average.r_finger_tip]
        return average


if __name__ == '__main__':
                
    rospy.init_node('sense_contact') 
    threshold_poke = rospy.get_param('/sense_contact/threshold_poke', 2000)
    threshold_grab = rospy.get_param('/sense_contact/threshold_grab', 500)
    calibrator = CalibratePressure(10)
    srv = rospy.Service('calibrate_pressure', Empty_srv, calibrator.calibrate_pressure)
    publisher = PublishCalibratedPressures(calibrator.calibration, threshold_poke, threshold_grab)

