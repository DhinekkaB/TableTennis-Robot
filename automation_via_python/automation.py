#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import random

# Speed settings for the motors
MIN_MOTOR_SPEED = 1000
MAX_MOTOR_SPEED = 2000

# Predefined sets of motor speeds
sets = [
    (1350, 1450, 1500), (1400, 1400, 1450), (1450, 1450, 1350), (1550, 1400, 1350), (1450, 1400, 1350),
    (1450, 1400, 1450), (1600, 1650, 1640), (1500, 1450, 1440), (1500, 1440, 1450), (1550, 1500, 1530)
]

# Function to publish motor speeds
def publish_motor_speeds(pub, mode):
    if mode == "STOP":
        speeds = f"{MIN_MOTOR_SPEED},{MIN_MOTOR_SPEED},{MIN_MOTOR_SPEED}"
    else:
        set_index = random.randint(0, len(sets) - 1)

        if mode == "BACKWARD":
            upperVal = sets[set_index][0]
            lowerVal = sets[set_index][1] + 200
            sideVal = sets[set_index][2]
        elif mode == "FORWARD":
            upperVal = sets[set_index][0] + 200
            lowerVal = sets[set_index][1]
            sideVal = sets[set_index][2]
        elif mode == "SIDE_SPIN":
            upperVal = sets[set_index][0]
            lowerVal = sets[set_index][1]
            sideVal = sets[set_index][2] + 200

        speeds = f"{upperVal},{lowerVal},{sideVal}"
    pub.publish(speeds)
    rospy.loginfo(f"Published motor speeds: {speeds}")

# Function to publish DC motor control commands
def publish_dc_motor_control(pub, command):
    pub.publish(command)
    rospy.loginfo(f"Published DC motor control command: {command}")

# Function to publish servo angle commands
def publish_servo_angle(pub, direction):
    if direction == 'right':
        angle = random.randint(75, 90)
    elif direction == 'left':
        angle = random.randint(90,105)
    else:
        rospy.logwarn("Invalid direction for servo angle. Use 'left' or 'right'.")
        return
    
    pub.publish(str(angle))
    rospy.loginfo(f"Published servo angle: {angle}")

def main():
    rospy.init_node('motor_publisher', anonymous=True)
    motor_pub = rospy.Publisher('motor_speeds_topic', String, queue_size=10)
    dc_motor_pub = rospy.Publisher('dc_motor_control_topic', String, queue_size=10)
    servo_pub = rospy.Publisher('servo_angle_topic', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    try:
        while not rospy.is_shutdown():
            user_input = input("Enter command (b: BACKWARD, f: FORWARD, s: SIDE SPIN, stop: Stop BLDC motors, dc_start: DC motor start, dc_stop: DC motor stop, left: Left Side, right: Right Side): ").strip().lower()
            if user_input in ['b', 'f', 's', 'stop']:
                mode = {"b": "BACKWARD", "f": "FORWARD", "s": "SIDE_SPIN", "stop": "STOP"}[user_input]
                publish_motor_speeds(motor_pub, mode)
            elif user_input in ['dc_start', 'dc_stop']:
                command = {"dc_start": "start", "dc_stop": "stop"}[user_input]
                publish_dc_motor_control(dc_motor_pub, command)
            elif user_input == 'left':
                publish_servo_angle(servo_pub,user_input)
            elif user_input == 'right':
                publish_servo_angle(servo_pub, user_input)
            else:
                rospy.logwarn("Invalid command. Please enter a valid command.")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()