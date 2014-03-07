#!/usr/bin/env python

import roslib
roslib.load_manifest('use_knob')

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from use_knob.srv import SendMarker


class PublishMarkers():
    def __init__(self):
        self.srv = rospy.Service('send_Marker', SendMarker, self.get_marker)
        self.array = MarkerArray()
        self.publish_markers()  # stand by to publish markers once received

    def get_marker(self, request):
        self.array.markers.append(request.marker)
        rospy.loginfo('Got a marker!')
        return True

    def publish_markers(self):
        pub = rospy.Publisher('/find_knob/markers', MarkerArray)
        while not rospy.is_shutdown():
            if self.array.markers:
                pub.publish(self.array)
            rospy.Rate(1.0)  # loop at 1.0Hz


if __name__ == '__main__':
    rospy.init_node('publish_Markers')
    publisher = PublishMarkers()

