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
Q11 = [139.12*pi/180.0, -44.71*pi/180.0, 96.22*pi/180.0, -139.60*pi/180.0, -90.68*pi/180.0, 18.99*pi/180.0] # base block
Q12 = [139.21*pi/180.0, -50.07*pi/180.0, 95.71*pi/180.0, -133.71*pi/180.0, -90.68*pi/180.0, 19.11*pi/180.0] # top block 
Q13 = [139.21*pi/180.0, -55.35*pi/180.0, 93.81*pi/180.0, -126.54*pi/180.0, -90.70*pi/180.0, 19.13*pi/180.0] # top block 

# Hanoi tower location 2
Q21 = [152.93*pi/180.0, -48.72*pi/180.0, 107.31*pi/180.0, -146.61*pi/180.0, -90.20*pi/180.0, 32.77*pi/180.0] # base block
Q22 = [152.94*pi/180.0, -55.44*pi/180.0, 106.63*pi/180.0, -139.21*pi/180.0, -90.22*pi/180.0, 32.80*pi/180.0] # second block
Q23 = [152.93*pi/180.0, -61.15*pi/180.0, 104.79*pi/180.0, -131.66*pi/180.0, -90.22*pi/180.0, 32.82*pi/180.0] # top block 

# Hanoi tower location 3
Q31 = [168.91*pi/180.0, -49.53*pi/180.0, 108.31*pi/180.0, -149.11*pi/180.0, -90.02*pi/180.0, 48.70*pi/180.0] # base block
Q32 = [168.90*pi/180.0, -55.92*pi/180.0, 107.63*pi/180.0, -141.84*pi/180.0, -90.04*pi/180.0, 48.72*pi/180.0] # second block
Q33 = [168.90*pi/180.0, -61.64*pi/180.0, 105.81*pi/180.0, -134.30*pi/180.0, -90.05*pi/180.0, 48.74*pi/180.0] # top block 

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
    global Q, suction_on, suction_off, digital_in_0, home

    ### Hint: Use the Q array to map out your towers by location and "height".
    try:
        src = Q[start_loc][start_height]
        des = Q[end_loc][end_height]    
    except IndexError:
        rospy.logerr('Incorrect indices')
        return 1
    
    error = 0
    safety_lift = 0.2   #radians for vertical lift

    safety_src = copy.deepcopy(src)
    safety_src[1] -= safety_lift

    safety_des = copy.deepcopy(des)
    safety_des[1] -= safety_lift

    error = move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)    
    if error != 0:
        rospy.logerr('Failed to reach home position')
        return error
    
    error = move_arm(pub_cmd, loop_rate, safety_src, 2.0, 2.0)    
    if error != 0:
        rospy.logerr('Failed to reach safety source position')
        return error
    
    error = move_arm(pub_cmd, loop_rate, src, 0.5, 0.5)    
    if error != 0:
        rospy.logerr('Failed to reach source position')
        return error


    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)

    if digital_in_0 != 1:
        rospy.logerr('Failed to grip block')
        return 2

    error = move_arm(pub_cmd, loop_rate, safety_src, 2.0, 2.0)    
    if error != 0:
        rospy.logerr('Failed to lift from source position')
        return error

    error = move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    if error != 0:
        rospy.logerr('Failed to reach home position')
        return error
    
    error = move_arm(pub_cmd, loop_rate, safety_des, 2.0, 2.0)
    if error != 0:
        rospy.logerr('Failed to reach safety destination position')
        return error
    
    error = move_arm(pub_cmd, loop_rate, des, 0.5, 0.5)
    if error != 0:
        rospy.logerr('Failed to reach destination position')
        return error

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)

    if digital_in_0 != 0:
        rospy.logerr('Failed to drop block')
        return 3

    error = move_arm(pub_cmd, loop_rate, safety_des, 2.0, 2.0)
    if error != 0:
        rospy.logerr('Failed to lift from destination position')
        return error
    
    error = move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    if error != 0:
        rospy.logerr('Failed to reach home position')
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

    error = 0
    def get_moves(n, source, dest, aux):
        if n==1:
            return [(source, dest)]
        
        else:
            moves = []
            moves.extend(get_moves(n-1, source, aux, dest))
            moves.append((source, dest))
            moves.extend(get_moves(n-1, aux, dest, source))
            return moves
        
        
    def execute_moves(move_source, move_dest, stacks):
        if not stacks[move_source]:
            rospy.logerr(f"No block on tower: {move_source}")
            return 1
        
        moving_block = stacks[move_source][-1]
        start_height = len(stacks[move_source]) - 1
        end_height = len(stacks[move_dest])

        #Check if the move is legal (if destination is empty or if the top block is larger)
        if stacks[move_dest] and stacks[move_dest][-1] < moving_block:
            rospy.logerr(f"Illegal move: Cannot place block {moving_block} on block {stacks[move_dest][-1]}")
            return 2
        
        #move_arm(pub_command, loop_rate, home, 4.0, 4.0)
        return_code = move_block(pub_command, loop_rate, start_loc=move_source, start_height=start_height, end_loc=move_dest, end_height=end_height)

        if return_code != 0:
            return return_code
        
        #update the stack
        moved_block = stacks[move_source].pop()
        stacks[move_dest].append(moved_block)

        rospy.loginfo(f"Moved block {moved_block} from tower {move_source} to tower {move_dest}")

        return 0

        # start_height = len(stacks[move_source]) - 1
        # end_height = len(stacks[move_dest])

        # if start_height < 0:
        #     rospy.logerr(f"No blockis on tower: {source}")
        #     return 1
        
        # moving_block = stacks[source][start_height]

        # if stacks[dest] and stacks[dest][-1] < moving_block:
        #     rospy.logerr(f"Illegal move")
        #     return 2
        
        # move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        # return_code = move_block(pub_command, loop_rate, start_loc=move_source, start_height=start_height, end_loc=move_dest, end_height=end_height)

        # if return_code != 0:
        #     return return_code
        
        # stacks[dest].append(stacks[source].pop())

        # gripper(pub_command, loop_rate, suction_off)

        # return 0

    #initialize the stacks
    stacks = [[], [], []]

    while True:
        try:
            start_location = int(input("Enter tower starting location: (0,1,2) or 3 to quit:").strip())
            if start_location == 3:
                sys.exit()

            end_location = int(input("Enter tower end location: (0,1,2) or 3 to quit:").strip())
            if end_location == 3:
                sys.exit()
            
            if not(0 <= start_location <= 2 and 0 <= end_location <= 2) or end_location == start_location:
                print("Invalid Input")
                continue
            
            if start_location == end_location:
                print("Start and End location cannot be the same")

            if not stacks[start_location]:
                print("no blocks on that starting tower")

            break

        except ValueError:
            print("Invalid Input")


    stacks[start_location] = [3, 2, 1] #3 stands for top and 1 for bottom block

    num_blocks = len(stacks[start_location])    
    auxiliary = 3 - start_location - end_location

    rospy.loginfo(f"Solving for Tower of Hanoi: {num_blocks} blocks from tower {start_location} to tower {end_location}")

    moves = get_moves(num_blocks, start_location, end_location, auxiliary)

    for i, (move_source, move_dest) in enumerate(moves):
        rospy.loginfo(f"Executing move {i+1}: from tower {move_source} to tower {move_dest}")
        error = execute_moves(move_source, move_dest, stacks)

        if error != 0:
            rospy.logerr(f"Failed to execute move{i+1}, error code: {error}")
            break
        else:
            rospy.loginfo(f"Move {i+1} completed")
    
    if error == 0:
        rospy.loginfo("Tower of Hanoi solved successfully")
        

        



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
