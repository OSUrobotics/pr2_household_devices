#!/usr/bin/env python

import roslib
roslib.load_manifest('sense_contact')

import rospy
from visualization_msgs.msg import Marker
from actionlib import SimpleActionClient, GoalStatus
from std_msgs.msg import Empty


class ContactMarkerHandler():
    """ Handles Marker msgs for left and right arms indicating gripper contact. """
    def __init__(self):
        """ Initialize Marker msgs, subscribers, and publishers. """
        self.text = 'CONTACT!'

        self.pubs = {}
        self.pubs['right'] = rospy.Publisher('visualization_marker', Marker)

        self.markers = {}
        self.markers['left'] = Marker()
        self.markers['right'] = Marker()
        for side, marker in self.markers.iteritems():
            """
            marker.header.frame_id = '/base_link'
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.position.x = 0.5
            marker.pose.position.y = 0.0
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.10
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            """
            marker.header.frame_id = '/r_wrist_roll_link'
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.05
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.10
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        self.pubs['right'].publish( self.markers['right'] )

        rospy.Subscriber('r_gripper_contact', Empty, self.update_and_publish_markers)


    def update_and_publish_markers(self, nothing):
        rospy.loginfo('Contact!')
        self.markers['right'].text = self.text
        self.markers['right'].color.a = 1.0

        hz = 10  # publish at 10hz
        r = rospy.Rate(hz)
        for n in range(2 * hz):  # for 2s
            self.markers['right'].color.a -= 1.0/20
            self.pubs['right'].publish( self.markers['right'] )
            r.sleep()
        


if __name__ == '__main__':
                
    rospy.init_node('contact_markers') 
    handler = ContactMarkerHandler()
    rospy.spin()
