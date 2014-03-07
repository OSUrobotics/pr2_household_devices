#!/usr/bin/env python

import roslib
roslib.load_manifest('cartesian_motion')

import rospy
import tf
import numpy
import copy
from geometry_msgs.msg import PoseStamped
from robot_mechanism_controllers.msg import JTCartesianControllerState
from math import sqrt, pi, sin, cos
from std_msgs.msg import Empty
from cartesian_motion.srv import *


class CartesianMover():
    """ Employ a JTCartesian controller to move the gripper. """
    def __init__(self):
        self.keep_going = True  # switches to False when gripper contacts something

        self.listener = tf.TransformListener()

        self.goal = self.get_current_poses('/base_link')

        self.sub = rospy.Subscriber('r_gripper_contact', Empty, self.contact_callback)

        self.pub = []
        self.pub.append( rospy.Publisher('l_cart/command_pose', PoseStamped) )
        self.pub.append( rospy.Publisher('r_cart/command_pose', PoseStamped) )

    def get_current_poses(self, frame):
        """ Get current gripper poses. """
        self.need_pose = [True, True]
        self.goal_raw = [None, None]
        rospy.Subscriber('l_cart/state', JTCartesianControllerState, self.pose_callback_left)
        rospy.Subscriber('r_cart/state', JTCartesianControllerState, self.pose_callback_right)
        while any(self.need_pose):  # wait until have poses of BOTH grippers
            rospy.sleep(0.01)
        goals = []
        for goal_raw in self.goal_raw:
            goal_raw.header.stamp = rospy.Time(0)
            self.listener.waitForTransform(frame, goal_raw.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            goals.append( self.listener.transformPose(frame, goal_raw) )
        rospy.loginfo('Got current poses!')
        return goals

    def pose_callback_left(self, pose):
        """ Callback for getting current LEFT gripper pose from controller. """
        self.goal_raw[0] = pose.x
        self.need_pose[0] = False

    def pose_callback_right(self, pose):
        """ Callback for getting current RIGHT gripper pose from controller. """
        self.goal_raw[1] = pose.x
        self.need_pose[1] = False

    def move_at_speed(self, end, speed, is_continuous):
        """ Move from current goal to input destination at input speed (in m/s).
        Input 'is_continuous' is a flag; motion stops at goal if it's false. """
        self.keep_going = True  # reset flag

        if self.which_arm == 'left':
            start = self.goal[0]
        if self.which_arm == 'right':
            start = self.goal[1]

        offset = self.add_PoseStamped(end, start, -1)  # find offset
        dist_xyz = offset.pose.position.__getstate__()
        dist_mag = sqrt(sum([d**2 for d in dist_xyz]))
        time = dist_mag / speed

        rate = 10.0  # in Hz
        if time == 0:
            scale = 1000  # arbitrarily large
            counter = 1
        else:
            scale = (1/rate) / time  # move by this fraction of the total path
            if scale > 1.0:
                scale = 1.0
                counter = 1
            else:
                counter = time / (1/rate)  # number of steps to achieve goal

        r = rospy.Rate( rate )
        while self.keep_going:
            start = self.add_PoseStamped(start, offset, scale)
            start.pose.orientation = end.pose.orientation  # achieve orientation right away
            self.go(start, 0.0)
            counter -= 1.0
            if counter <= 0 and not is_continuous:
                self.keep_going = False  # stop after one iteration
            r.sleep()

        poses_final = self.get_current_poses('/base_link')
        if self.which_arm == 'left':
            pose_final = poses_final[0]
        if self.which_arm == 'right':
            pose_final = poses_final[1]
        
        return pose_final  # return final position

    def add_PoseStamped(self, start, offset, scale):
        """ Add input 'offset' to input 'start' after multiplying by input 'scale'.
        Do subtraction by passing -1 as 'scale' """
        final = PoseStamped()
        final.header = start.header

        # Positions
        pos_s = start.pose.position.__getstate__()
        pos_o = offset.pose.position.__getstate__()
        pos_f = [s + scale*o for s, o in zip(pos_s, pos_o)]
        for el, f in zip('xyz', pos_f):
            final.pose.position.__setattr__(el, f)

        # Orientations
        or_s = start.pose.orientation.__getstate__()
        or_o = offset.pose.orientation.__getstate__()
        or_f = [s + scale*o for s, o in zip(or_s, or_o)]
        for el, f in zip('xyzw', or_f):
            final.pose.orientation.__setattr__(el, f)
            
        return final

    def go(self, goal, wait):
        """ Publish pose goal, maybe wait a tick """
        if self.which_arm == 'left':
            self.goal[0] = goal
            self.pub[0].publish( goal )
        elif self.which_arm == 'right':
            self.goal[1] = goal
            self.pub[1].publish( goal )
        rospy.sleep( wait )

    def get_stored_pose(self, name):
        """ Hard-coded pose data for UNTUCK and READY positions """
        goal = PoseStamped()
        goal.header.frame_id = '/base_link'
        
        if name == 'UNTUCK':
            goal.pose.position.x = 0.388
            goal.pose.position.y = -0.373
            goal.pose.position.z = 0.693
            goal.pose.orientation.x = -0.108
            goal.pose.orientation.y = -0.533
            goal.pose.orientation.z = -0.167
            goal.pose.orientation.w = 0.822

        if name == 'READY':
            goal.pose.position.x =  0.40
            goal.pose.position.y = -0.15
            goal.pose.position.z =  1.15
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            
        if self.which_arm == 'left':
            goal.pose.position.y *= -1
            goal.pose.orientation.x *= -1
            goal.pose.orientation.z *= -1

        return goal
        
    def contact_callback(self, foo):
        """ When gripper contacts something, switch from moving forward to standing off. """
        rospy.loginfo('GRIPPER_MOVER> Contact detected!')
        self.made_contact = True
        self.keep_going = False
        
    def move_gripper(self, request):
        """ Move to goal in response to service request. """
        self.made_contact = False

        ### Handle request fields ###
        self.which_arm = str(request.which_arm)
        if request.stored_goal:
            rospy.loginfo('GRIPPER_MOVER> Retrieving stored goal: {0}'.format(request.stored_goal))
            goal = self.get_stored_pose( request.stored_goal )
        else:
            goal = request.pose_goal
        goal_offset = request.goal_offset
        """
        if goal.header.frame_id != goal_offset.header.frame_id:
            rospy.logerr('GRIPPER_MOVER> Offset frame is not identical to goal frame! Aborting.')
            response = MoveGripperResponse()
            response.success = False
            return response
        else:
        """
        goal.pose.position.x += goal_offset.vector.x
        goal.pose.position.y += goal_offset.vector.y
        goal.pose.position.z += goal_offset.vector.z

        ### Transform goal to base_link if necessary ###
        frame = '/base_link'
        if goal.header.frame_id != frame:
            goal.header.stamp = rospy.Time(0)
            self.listener.waitForTransform(frame, goal.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            goal = self.listener.transformPose(frame, goal)

        ### Do movement ###
        final_pose = self.move_at_speed(goal, request.speed, False)

        ### Transform final_pose back to device frame if necessary ###
        if not request.stored_goal:
            frame_device = request.pose_goal.header.frame_id
            final_pose.header.stamp = rospy.Time(0)
            self.listener.waitForTransform(frame_device, final_pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            final_pose = self.listener.transformPose(frame_device, final_pose)

        ### Construct response ###
        response = MoveGripperResponse()
        response.final_pose = final_pose
        response.made_contact = self.made_contact
        response.success = True
        return response


    def turn_gripper(self, request):
        """ Turn gripper in response to service request. """
        ### Handle request fields ###
        self.which_arm = str(request.which_arm)
        radians_CW = request.degrees_CW * pi / 180

        # 1) Get current pose in /r_wrist_roll_link
        frame = '/' + self.which_arm[0] + '_wrist_roll_link'
        poses_current = self.get_current_poses(frame)
        if self.which_arm == 'left':
            pose_current = poses_current[0]
        if self.which_arm == 'right':
            pose_current = poses_current[1]
        q_old = numpy.array([pose_current.pose.orientation.__getattribute__(key) for key in 'xyzw'])

        # 2) Form rotation quaternion
        q_turn = numpy.array([el*sin(radians_CW/2) for el in [1, 0, 0]])
        q_turn = numpy.append(q_turn, cos(radians_CW/2))  # rotation angle

        # 3) Multiply quaternions
        q_new = multiply_quaternions(q_old, q_turn)

        # 4) Update pose goal in /r_wrist frame
        pose_new = copy.deepcopy(pose_current)
        for value, axis in zip(q_new, 'xyzw'):
            pose_new.pose.orientation.__setattr__(axis, value)

        # 5) Convert pose goal back to /base_link
        frame_base = '/base_link'
        pose_new.header.stamp = rospy.Time(0)
        self.listener.waitForTransform(frame_base, pose_new.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose_new_tf = self.listener.transformPose(frame_base, pose_new)

        # 6) self.go(pose_goal, wait)
        self.go(pose_new_tf, 0.0)

        ### Construct response ###
        response = TurnGripperResponse()
        response.success = True
        return response


def multiply_quaternions(q1, q2):
    """ Multiply two quaternions. In terms of rotations, does first rotation followed by second one.
    INPUTS: two four-element numpy vectors of form [x, y, z, w] """
    s = q1[3]; v = q1[:3]
    t = q2[3]; w = q2[:3]
    # Equation: q1*q2 = (s+v_bar)*(t+w_bar) = (s*t - v DOT w) + (s*w + t*v + v CROSS w)
    q_vector = s*w + t*v + numpy.cross(v, w)
    q_scalar = s*t - numpy.dot(v, w)
    q_product = numpy.append(q_vector, q_scalar)
    return q_product


if __name__ == '__main__':

    rospy.init_node('cartesian_motion')
    mover = CartesianMover()
    srv1 = rospy.Service('move_gripper', MoveGripper, mover.move_gripper)
    srv2 = rospy.Service('turn_gripper', TurnGripper, mover.turn_gripper)
    rospy.spin()
