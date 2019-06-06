#!/usr/bin/env python

import sys
from math import pi

import rospy
import moveit_ros_planning_interface
import moveit_commander

from geometry_msgs.msg import TwistStamped
from leap_motion.msg import Hand, Human

class MovingRobot(object):
    def leapData(self, leap_msg):
        rightIsPresent = leap_msg.right_hand.is_present
        
        if rightIsPresent:
            hand = leap_msg.right_hand
            position = hand.palm_center

            rpy = [-hand.yaw, -hand.roll, -hand.pitch]
            self.movement(position, rpy)
    
    def movement(self, position, rpy):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()

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

        if twist.twist.angular.x > 3.14:
            twist.twist.angular.x = (-rpy[0] + prevAng.x) - pi

        if twist.twist.angular.y > 3.14:
            twist.twist.angular.y = (-rpy[1] + prevAng.y) - pi

        if twist.twist.angular.x > 3.14:
            twist.twist.angular.z = (-rpy[2] + prevAng.z) - pi

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
        
        rospy.loginfo(rpy[0])
        rospy.loginfo(twist.twist)
        self.jogPub.publish(twist)
    
    def start_position(self):
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/3
        joint_goal[2] = pi/3
        joint_goal[3] = 0
        joint_goal[4] = pi/2
        joint_goal[5] = 0

        # Calling stop() ensures that there is no residual movement
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        rospy.loginfo("STARTING POSITION IS DONE")
        return move_group.get_current_pose().pose

    def __init__(self):
        self.jogPub = rospy.Publisher("jog_arm_server/delta_jog_cmds", 
            TwistStamped, queue_size=1)
        
        self.subscriber = rospy.Subscriber("/leap_motion/leap_device", Human, 
            self.leapData, queue_size=1)
        rospy.loginfo("CALLOUT LEAPDATA")
        rospy.spin()
        
if __name__ == '__main__':
    # First initialize 'moveit_commander' and a 'rospy' node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur5_single", anonymous=True)

    # set correct group name
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    rospy.loginfo("MAIN SETUP FINISHED")
    MovingRobot()