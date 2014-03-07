#!/usr/bin/env python

import roslib
roslib.load_manifest('use_knob')

import sys
import rospy
from moveit_commander import MoveGroupCommander


class HeadMover():
    """ Moves head to specified joint values """
    def __init__(self):
        self.group_head = MoveGroupCommander('head')
        
    def move_head(self, name, joint_values):
        rospy.loginfo('Moving head to specified position')
        self.group_head.set_joint_value_target(joint_values)
        self.group_head.go()


if __name__ == '__main__':

    if len(sys.argv) == 3:
        
        rospy.init_node('move_head')
        
        # Move head into position
        head_mover = HeadMover()
        head_mover.move_head('READY', [float(sys.argv[1]), float(sys.argv[2])])

    else: 
        
        print 'Please enter two arguments specifying the desired head position.'
