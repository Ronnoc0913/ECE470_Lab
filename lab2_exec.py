#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *



# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
Q11 = [140.46*pi/180.0, -44.96*pi/180.0, 97.92*pi/180.0, -141.07*pi/180.0, -90.61*pi/180.0, 20.32*pi/180.0] # base block
Q12 = [140.46*pi/180.0, -50.41*pi/180.0, 97.35*pi/180.0, -135.06*pi/180.0, -90.61*pi/180.0, 20.31*pi/180.0] # second block
Q13 = [140.46*pi/180.0, -55.75*pi/180.0, 95.58*pi/180.0, -127.93*pi/180.0, -90.63*pi/180.0, 20.35*pi/180.0] # top block 

# Hanoi tower location 2
Q21 = [154.5*pi/180.0, -49.12*pi/180.0, 107.86*pi/180.0, -146.79*pi/180.0, -90.13*pi/180.0, 34.32*pi/180.0] # base block
Q22 = [154.5*pi/180.0, -54.64*pi/180.0, 107.37*pi/180.0, -149.78*pi/180.0, -90.15*pi/180.0, 34.34*pi/180.0] # second block
Q23 = [154.5*pi/180.0, -60.95*pi/180.0, 105.54*pi/180.0, -132.65*pi/180.0, -90.16*pi/180.0, 34.36*pi/180.0] # top block 

# Hanoi tower location 3
Q31 = [170.6*pi/180.0, -62.32*pi/180.0, 104.88*pi/180.0, -130.71*pi/180.0, -89.60*pi/180.0, 50.47*pi/180.0] # base block
Q32 = [170.6*pi/180.0, -54.67*pi/180.0, 107.34*pi/180.0, -140.81*pi/180.0, -89.59*pi/180.0, 50.44*pi/180.0] # second block
Q33 = [170.6*pi/180.0, -61.23*pi/180.0, 105.40*pi/180.0, -132.30*pi/180.0, -89.61*pi/180.0, 50.47*pi/180.0] # top block 

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0
analog_in_1 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def suction_callback(msg):
    global digital_in_0, analog_in_0, analog_in_1

    digital_in_0 = int(msg.DIGIN)
    analog_in_0 = float(msg.AIN0)
    analog_in_1 = float(msg.AIN1)



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q, suction_on, suction_off, digital_in_0

    ### Hint: Use the Q array to map out your towers by location and "height".
    try:
        src = Q[start_loc][start_height]
        des = Q[end_loc][end_height]    
    except IndexError:
        rospy.logerr('Incorrect indices')
        return 1
    
    error = 0

    safety_src = copy.deepcopy(src)
    safety_src[1] -= 0.5

    safety_des = copy.deepcopy(des)
    safety_des[1] -= 0.5

    error = move_arm(pub_cmd, loop_rate, src, 4.0, 4.0)    
    if error != 0:
        rospy.logerr('Failed to reach source position')
        return error

    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)

    if digital_in_0 != 1:
        rospy.logerr('Failed to grip block')
        return 2

    error = move_arm(pub_cmd, loop_rate, safety_src, 4.0, 4.0)    
    if error != 0:
        rospy.logerr('Failed to reach source position')
        return error

    error = move_arm(pub_cmd, loop_rate, des, 4.0, 4.0)
    if error != 0:
        rospy.logerr('Failed to reach destination position')
        return error

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)

    if digital_in_0 != 0:
        rospy.logerr('Failed to drop block')
        return 3

    error = move_arm(pub_cmd, loop_rate, safety_des, 4.0, 4.0)
    if error != 0:
        rospy.logerr('Failed to reach destination position')
        return error

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, suction_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    # input_done = 0
    # loop_count = 0

    # while(not input_done):
    #     input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")


    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

   


    def get_moves(n, source, dest, aux):
        if n==1:
            return [(source, dest)]
        
        else:
            moves = []
            moves.extend(get_moves(n-1, source, dest, aux))
            moves.append([source, dest])
            moves.extend(get_moves(n-1, aux, dest, source))
            return moves
        
        
    def execute_moves(source, dest, stacks):
        start_height = len(stacks[move_source]) - 1
        end_height = len(stacks[move_dest])

        if start_height < 0:
            rospy.logerr(f"No blockis on tower: {source}")
            return 1
        
        moving_block = stacks[source][start_height]

        if stacks[dest] and stacks[dest][-1] < moving_block:
            rospy.logerr(f"Illegal move")
            return 2
        
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        return_code = move_block(pub_command, loop_rate, start_loc=move_source, start_height=start_height, end_loc=move_dest, end_height=end_height)

        if return_code != 0:
            return return_code
        
        stacks[dest].append(stacks[source].pop())

        gripper(pub_command, loop_rate, suction_off)

        return 0


    stacks = [[3,2,1], [], []]

    while True:
        try:
            start_location = int(input("Enter tower starting location: (0,1,2) or 3 to quit:").strip())
            if start_location == 3:
                sys.exit()

            end_location = int(input("Enter tower end location: (0,1,2) or 3 to quit:").strip())
            
            if not(0 <= start_location <= 2 and 0 <= end_location <= 2) or end_location == start_location:
                print("Invalid Input")
                continue
            
            if not stacks[start_location]:
                print("no blocks on that tower")

            break

        except ValueError:
            print("Invalid Input")


    num_blocks = len(stacks[start_location])

    auxiliary = 3 - start_location - end_location
    moves = get_moves(num_blocks, start_location, end_location, auxiliary)

    for i, (move_source, move_dest) in enumerate(moves):

        error = execute_moves(move_source, move_dest, stacks)

        if error != 0:
            rospy.logerr(f"Failed to execute move{i+1}, error code: {error}")
            break

        



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
