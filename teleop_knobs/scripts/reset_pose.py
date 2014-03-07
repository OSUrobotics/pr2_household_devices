#!/usr/bin/env python

import roslib
roslib.load_manifest('teleop_knobs')

import rospy
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib 
from cartesian_motion.srv import *

if __name__ == '__main__':

    rospy.init_node('reset_pose')

    # Move right arm to READY position
    rospy.wait_for_service('move_gripper')
    move_gripper = rospy.ServiceProxy('move_gripper', MoveGripper)
    request = MoveGripperRequest()
    request.which_arm = 'right'

    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.pose.position.x =  0.30
    p.pose.position.y = -0.35
    p.pose.position.z =  1.00
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 1.0
    request.pose_goal = p

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
    g.target.point.y = -0.3
    g.target.point.z =  0.9
    g.min_duration = rospy.Duration(1.0)
    
    client.send_goal(g)
    client.wait_for_result()
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        print "Succeeded"
    else:
        print "Failed"
