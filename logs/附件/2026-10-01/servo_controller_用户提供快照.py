#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

# pip3 install -r requirement.txt

LED_Pin = 'SPI1_MOSI'  # pin 19

kit = ServoKit(channels=16)

angle_front = 170  #初始角度
angle_down = 55   #识别角度
angle_open=120
angle_close=30

def cmd_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received message: %s", data.data)
    if data.data == 'Drop':
        kit.servo[1].angle = angle_open
        time.sleep(4)
        kit.servo[1].angle = angle_close
    elif data.data == 'PUT1':
        kit.servo[0].angle = 150
        time.sleep(2)
        kit.servo[0].angle = 10
    elif data.data == 'PUT2':
        kit.servo[1].angle = 130
        time.sleep(2)
        kit.servo[1].angle = 12
    elif data.data == 'PUT3':
        kit.servo[2].angle = 45
        time.sleep(2)
        kit.servo[2].angle = 175
    elif data.data == 'Front':
        kit.servo[0].angle = angle_front
    elif data.data == 'Down':
        kit.servo[0].angle = angle_down
    elif data.data == 'Blink':
        GPIO.output(LED_Pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_Pin, GPIO.LOW)


def input_callback(event):
    str = input()
    if 'a' in str:
        try:
            angle_in = int(str[1:])
            if (angle_in >= 0 and angle_in <= 180):
                rospy.loginfo('Set servo angle: %d.', angle_in)
                kit.servo[0].angle = angle_in
            else:
                rospy.loginfo('Range of Servo angle is: [0-180].')
        except:
            rospy.loginfo('Please input the command, such as: a120.')
    elif 'b' in str:
        try:
            angle_in = int(str[1:])
            if (angle_in >= 0 and angle_in <= 180):
                rospy.loginfo('Set servo angle: %d.', angle_in)
                kit.servo[1].angle = angle_in
            else:
                rospy.loginfo('Range of Servo angle is: [0-180].')
        except:
            rospy.loginfo('Please input the command, such as: a120.')
    elif str == 'f' or str == 'F':
        kit.servo[0].angle = angle_front
    elif str == 'd' or str == 'D':
        kit.servo[0].angle = angle_down

    elif str == 'o' or str == 'O':
        kit.servo[1].angle = angle_open
    elif str == 'c' or str == 'C':
        kit.servo[1].angle = angle_close

    elif str == 'l' or str == 'L':
        count = 3
        rospy.loginfo('Blink 3 times.')
        while (count > 0):
            GPIO.output(LED_Pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(LED_Pin, GPIO.LOW)
            time.sleep(1)
            count -= 1


if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('servo_controller_node', anonymous=True)

    rospy.Subscriber("/gpio_cmd", String, cmd_callback)

    rospy.Timer(rospy.Duration(0.5), input_callback)

    # GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_Pin, GPIO.OUT)
    GPIO.output(LED_Pin, GPIO.LOW)
    # mode = GPIO.getmode()

    kit.servo[0].angle = angle_front
    kit.servo[1].angle = angle_close

    print('\033[1;36m' + 'GPIO Control Init.' + '\033[0m')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    kit.servo[0].angle = angle_front
    GPIO.cleanup()
