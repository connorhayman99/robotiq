#!/usr/bin/env python3
import rospy
# unsure if I need this 
#from __future__ import print_function

#include <ros/ros.h>
#include <std_msgs/Float64.h>

# lets see if I can get the gripper to close from position 0 to 155
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput
from six.moves import input
from std_msgs.msg import String
from std_msgs.msg import Float64

def activate(command):
    """activate the gripper"""
    # you have to activate the gripper before you do anything, lines 18-22 is activation
    command = Robotiq3FGripperRobotOutput()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150


    return command

def move_to_0(command):
    """open the gripper"""
    command.rSPA = 25
    command.rPRA = int(0)
    
    return command

def move_to_255(command):
    """close the gripper"""
    command.rSPA = 25
    command.rPRA = int(255)

    return command

def move_to_120(command):
    """Move the gripper to position 120"""
    command.rSPA = 25
    command.rPRA = int(120)

    return command
  
def publisher():
    # initialise the node with its own name that I can chose
    rospy.init_node("myown")
    # define as publisher
    pub = rospy.Publisher("myown", Robotiq3FGripperRobotOutput, queue_size=10)
    command = Robotiq3FGripperRobotOutput()

    
    while not rospy.is_shutdown():
        command = activate(command)
        pub.publish(command)
        command_dos = move_to_0(command)
        pub.publish(command_dos)
        rospy.sleep(5)
        command_tres = move_to_120(command)
        pub.publish(command_tres)
        rospy.sleep(5)
        

if __name__ == "__main__":
    publisher()

