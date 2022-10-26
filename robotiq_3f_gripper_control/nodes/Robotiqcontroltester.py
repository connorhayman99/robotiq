#!/usr/bin/env python3
from pickle import FALSE
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
import collections
import time

# key: value
mass_force_dict = {'mass1': 0, 
                   'mass2': 0, 
                   'mass3': 0, 
                   'force1': 0, 
                   'force2': 0, 
                   'force3': 0}

# mass dictionary
mass_dict = {'mass1': 0, 
             'mass2': 0, 
             'mass3': 0}

# force dictionary
force_dict = {'force1': 0,
              'force2': 0,
              'force3': 0}

# list of mass values for sensor 1
mass_list1 = collections.deque(maxlen=10)

# list of mass values for sensor 2
mass_list2 = collections.deque(maxlen=10)

# list of mass values for sensor 3
mass_list3 = collections.deque(maxlen=10)

# list of force values for sensor 1
force_list1 = collections.deque(maxlen=10)

# list of force values for sensor 2
force_list2 = collections.deque(maxlen=10)

# list of force values for sensor 3
force_list3 = collections.deque(maxlen=10)

# highest_value
highest_value = 0

# mass of the object
mass = 0.045

# gravitational coefficient
g = 9.81

# threshold_value for fn
fn_threshold = 3.27

# fn_decrease
fn_decrease = -1

def activate(command):
    """activate the gripper"""
    # you have to activate the gripper before you do anything, lines 73-76 is activation
    #command = Robotiq3FGripperRobotOutput()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150


    return command


def publisher():
    # initialise the node with its own name that I can chose. I have called it "myown"
    rospy.init_node("myown")
    # define as publisher and topic is named "myown"
    pub = rospy.Publisher("myown", Robotiq3FGripperRobotOutput, queue_size=10)

    # topic is mass, mass2, mass3, force, force2, force3
    rospy.Subscriber("mass", Float64, callback1)
    rospy.Subscriber("mass2", Float64, callback2) 
    rospy.Subscriber("mass3", Float64, callback3)
    rospy.Subscriber("force", Float64, callback4)
    rospy.Subscriber("force2", Float64, callback5) 
    rospy.Subscriber("force3", Float64, callback6) 
    
    if not rospy.is_shutdown():

        # activate the gripper if needed, this is required upon start up
        # command put in this section rather than above the rospy.is_shutdown
        command = Robotiq3FGripperRobotOutput()
        command_one = activate(command)
        pub.publish(command_one)
        rospy.sleep(7)

        # line 106-108 resets the gripper back to a fully open position when the programme is re-run
        command.rSPA = 50
        command.rPRA = int(0)
        pub.publish(command)
        rospy.sleep(8)
               
        # The gripper is closed up until a certain force (that is specified in the while loop below, on line 121) is exceeded.
        command.rSPA = 1
        command.rPRA = int(255)
        pub.publish(command)
        rospy.loginfo("point reached")
                
        while True:
            def loop():
                for key, value in force_dict.items():
                    # the 3.27 value is chosen as the upper force limit for my experiments 
                    if value > fn_threshold:
                        print("It worked")
                        # return false gets the programme out of def loop() function
                        return False    
                return True
            if not loop():
                print("Lets break")
                break
        print("Exited")

        # now I want to move the fingers back by one position at a time, so the gripper is opening again
        command.rSPA = 1
        command.rPRA = int(0)
        pub.publish(command)
        
        
        # need to set previous_value and highest_value parameters before beginning the while loop otherwise 
        # an error message comes up saying they are undefined
        previous_value = 0
        highest_value = 0
        
        while True:

            if highest_value > 0:
                break
            for current_value in list(force_list2):
                print("current value is")
                print(current_value)
                # need to compare two values so if the total values are less than or equal to one begin
                # the iteration of the loop again.
                if len(force_list2) <=1:
                    continue
                
                # decerease of 1N indicating slip has occured
                if current_value - previous_value < fn_decrease:
                    print("slip occuring")
                    print("force_list")
                    print(force_list2)
                    print("highest_value")
                    highest_value = max(force_list2)
                    print(current_value)
                    # the point at which slip occured, number goes from highest
                    # to almost 0
                    Fn = highest_value
                    print("fn value")
                    print(Fn)
                    coefficient_of_friction = (mass * g ) / (3*Fn)
                    print("coefficient_of_friction")
                    print(coefficient_of_friction)
                    break
                
                print("previous value is")
                print(previous_value)
                previous_value = current_value 

        # close the gripper again to get a firm grip on the object
        print("gripper stop")
        command.rSPA = 1
        command.rPRA = int(255)
        pub.publish(command)
        

        while True:

            def stop():
                for big_num in list(force_list2):
                    # when the force applied is great than the previous highest value in 
                    # the last chunk of code then break out the loop
                    # of the loop
                    if big_num >= highest_value:
                        print(force_list2)
                        return False
                return True
            if not stop():
                break

        # get the gripper to stop
        print("stop the gripper")
        #command = Robotiq3FGripperRobotOutput()
        command.rSPA = 0
        command.rSPB = 0
        command.rSPC = 0
        pub.publish(command)
        print("Programme finished")

             
        print("Final recorded masses")
        # prints out the first mass value in the respective deque and the last mass value
        print(mass_list1[0])
        print(mass_list1[-1])
        print(mass_list1)
        print(mass_list2[0])
        print(mass_list2[-1])
        print(mass_list2)
        print(mass_list3[0])
        print(mass_list3[-1])
        print(mass_list3)
        print("Final recorded forces")
        # prints out the first force value in the respective deque and the last force value
        print(force_list1[0])
        print(force_list1[-1])
        print(force_list1)
        print(force_list2[0])
        print(force_list2[-1])
        print(force_list2)
        print(force_list3[0])
        print(force_list3[-1])
        print(force_list3)


  

def callback1(data):
    # I've used the callback function to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo(mass_force_dict)
    #rospy.loginfo("The predicted mass of sensor 1 is %s", data.data)
    mass_dict["mass1"] = data.data
    mass_list1.append(data.data)
    #rospy.loginfo("This is mass deque 1")
    #rospy.loginfo(mass_list1)


def callback2(data):
    # I've used the callback funuction to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo("The predicted mass of sensor 2 is %s", data.data)
    mass_dict["mass2"] = data.data
    mass_list2.append(data.data)
    # rospy.loginfo("This is mass deque 2")
    # rospy.loginfo(mass_list2)

                       
def callback3(data):
    # I've used the callback funuction to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo("The predicted mass of sensor 3 is %s", data.data)
    mass_dict["mass3"] = data.data
    mass_list3.append(data.data)
    # rospy.loginfo("This is mass deque 3")
    # rospy.loginfo(mass_list3)


def callback4(data):
    # I've used the callback funuction to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo("The predicted force of sensor 1 is %s", data.data)
    force_dict["force1"] = data.data
    force_list1.append(data.data)
    # rospy.loginfo("This is force deque 1")
    # rospy.loginfo(force_list1)


def callback5(data):
    # I've used the callback funuction to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo("The predicted force of sensor 2 is %s", data.data)
    force_dict["force2"] = data.data
    force_list2.append(data.data)
    # rospy.loginfo("This is force deque 2")
    # rospy.loginfo(force_list2)


def callback6(data):
    # I've used the callback funuction to show me that the values that I'm getting from the FSRs
    # which is being sent via rosserial is working correctly
    #rospy.loginfo("The predicted force of sensor 3 is %s", data.data)
    force_dict["force3"] = data.data
    force_list3.append(data.data)
    # rospy.loginfo("This is force deque 3")
    # rospy.loginfo(force_list3)

if __name__ == "__main__":
    publisher()
