#!/usr/bin/env python

import sys
from math import pi

import rospy
import moveit_ros_planning_interface
import moveit_commander

from geometry_msgs.msg import TwistStamped, Pose
from leap_motion.msg import Hand, Human

class MovingRobot(object):
    def leapData(self, leap_msg):
        leftIsPresent = leap_msg.left_hand.is_present
        rightIsPresent = leap_msg.right_hand.is_present

        if leftIsPresent:
            hand = leap_msg.left_hand
            position = hand.palm_center
            rpy = [-hand.pitch, hand.yaw, -hand.roll]
            self.movement(position, rpy, 0)

        if rightIsPresent:
            hand = leap_msg.right_hand
            position = hand.palm_center
            rpy = [-hand.pitch, hand.yaw, -hand.roll]
            self.movement(position, rpy, 1)
    
    def movement(self, position, rpy, h):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()

        if h == 0:
            move_group = self.move_group_left
        else:
            move_group = self.move_group_right
        
        prevPos = move_group.get_current_pose().pose.position
        prevAng = move_group.get_current_pose().pose.orientation

        minChange = 0.05
        
        # To make sure that x is in positive half
        consForPos = 0.5
        handMultiplier = -2

        # leap is upside down so positions are changed (z->x, x->y, y->z)
        # X-axis
        temp = (handMultiplier * position.z) - prevPos.x + consForPos
        
        if temp > 1:
            twist.twist.linear.x = 1
        elif temp < -1:
            twist.twist.linear.x = -1
        elif temp < minChange and temp > -minChange:
            samePosX = round(prevPos.x, 2) == round(handMultiplier 
                * position.z, 2)
            if samePosX:
                twist.twist.linear.x = 0
            if temp <= 0:
                twist.twist.linear.x = -minChange
            else:
                twist.twist.linear.x = minChange
        else:
            twist.twist.linear.x = temp

        # Y-axis
        temp = handMultiplier * position.x - prevPos.y

        if temp > 1:
            twist.twist.linear.y = 1
        elif temp < -1:
            twist.twist.linear.y = -1
        elif temp < minChange and temp > -minChange:
            samePosY = round(prevPos.y, 2) == round(handMultiplier 
                * position.x, 2)
            if samePosY:
                twist.twist.linear.y = 0
            elif temp <= 0:
                twist.twist.linear.y = -minChange
            else:
                twist.twist.linear.y = minChange
        else:
            twist.twist.linear.y = temp

        #Z-axis
        temp = 1.5 * position.y - prevPos.z

        if temp > 1:
            twist.twist.linear.z = 1
        elif temp < -1:
            twist.twist.linear.z = -1
        elif temp < minChange and temp > -minChange:
            samePosZ = round(prevPos.z, 2) == round(1.5 
                * position.y, 2)
            if samePosZ:
                twist.twist.linear.z = 0
            elif temp <= 0:
                twist.twist.linear.z = -minChange
            else:
                twist.twist.linear.z = minChange
        else:
            twist.twist.linear.z = temp
        

        twist.twist.angular.x = (rpy[0] - prevAng.x)
        twist.twist.angular.y = (rpy[1] - prevAng.y)
        twist.twist.angular.z = (rpy[2] - prevAng.z)

        if twist.twist.angular.x > 1:
            twist.twist.angular.x = 1
        elif twist.twist.angular.x < -1:
            twist.twist.angular.x = -1
        if twist.twist.angular.y > 1:
            twist.twist.angular.y = 1
        elif twist.twist.angular.y < -1:
            twist.twist.angular.y = -1
        if twist.twist.angular.z > 1:
            twist.twist.angular.z = 1
        elif twist.twist.angular.z < -1:
            twist.twist.angular.z = -1

        rospy.loginfo(twist.twist)

        if h == 0:
            self.jogPubL.publish(twist)
        else:
            self.jogPubR.publish(twist)
    

    def __init__(self):
        self.move_group_left = moveit_commander.MoveGroupCommander("left_arm")
        self.move_group_right = moveit_commander.MoveGroupCommander("right_arm")

        self.jogPubR = rospy.Publisher("right_jog_arm_server/delta_jog_cmds", 
            TwistStamped, queue_size=1)
        self.jogPubL = rospy.Publisher("left_jog_arm_server/delta_jog_cmds", 
            TwistStamped, queue_size=1)

        self.subscriber = rospy.Subscriber("/leap_motion/leap_device", Human, 
            self.leapData, queue_size=1)
        rospy.loginfo("CALLOUT LEAPDATA")
        rospy.spin()
        
if __name__ == '__main__':
    # First initialize 'moveit_commander'_ and a 'rospy'_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_arm", anonymous=True)

    # set correct group name
    rospy.loginfo("MAIN SETUP FINISHED")
    MovingRobot()