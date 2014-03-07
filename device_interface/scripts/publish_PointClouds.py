#!/usr/bin/env python

import roslib
roslib.load_manifest('use_knob')

import rospy
from sensor_msgs.msg import PointCloud2
from use_knob.srv import SendPointCloud


class PublishClouds():
    def __init__(self):
        self.srv = rospy.Service('send_PointCloud', SendPointCloud, self.get_cloud)
        self.publishers = []
        self.clouds = []
        self.publish_clouds()  # stand by to publish point clouds once received

    def get_cloud(self, request):
        self.publishers.append(rospy.Publisher(request.cloud_topic, PointCloud2))
        self.clouds.append(request.cloud)
        rospy.loginfo('Got a point cloud!')
        return True

    def publish_clouds(self):
        while not rospy.is_shutdown():
            if self.clouds:
                for cloud, pub in zip(self.clouds, self.publishers):
                    cloud.header.stamp = rospy.Time.now()-rospy.Time(1)  # HACK: update time stamp to avoid TF issues
                    pub.publish(cloud)
            rospy.Rate(1.0)  # loop at 1.0Hz


if __name__ == '__main__':
    rospy.init_node('publish_PointClouds')
    publisher = PublishClouds()

