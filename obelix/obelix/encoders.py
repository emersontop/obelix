#!/usr/bin/env python3

# Node information:
# Name: encoders
# Description: Counts encoders pulses

# importing libraries
import RPi.GPIO as GPIO        
import rospy
from std_msgs.msg import Float32MultiArray

contar_pulso = 0

# Setup the pin numbers

# For the right encoder
ENC_R1 = 3
ENC_R2 = 4

# For the left encoder
ENC_L1 = 15
ENC_L2 = 18

class encoders:

    def __init__(self):

        ###########
        # Publisher

        self.pub = rospy.Publisher('/base/encoders', Float32MultiArray, queue_size=1)
        self.lr_counter = Float32MultiArray()
        self.lr_counter.data = [0]*2

        ###################
        # Configure the pin

        GPIO.setmode(GPIO.BCM)

        # Right encoder
        GPIO.setup(ENC_R1,GPIO.IN)
        GPIO.setup(ENC_R2,GPIO.IN)
        GPIO.add_event_detect(ENC_R1,GPIO.RISING,callback=self.counter_er1)

        # Left encoder
        GPIO.setup(ENC_L1,GPIO.IN)
        GPIO.setup(ENC_L2,GPIO.IN)
        GPIO.add_event_detect(ENC_L1,GPIO.RISING,callback=self.counter_el1)

    # Definition of the callback functions
    def counter_el1(self, channel):
        # self.left_counter +=1
        if GPIO.input(ENC_L2):
            self.lr_counter.data[0] +=1
        else:
            self.lr_counter.data[0] -=1

    def counter_er1(self, channel):
        # self.left_counter +=1
        if GPIO.input(ENC_R2):
            self.lr_counter.data[1] -=1
        else:
            self.lr_counter.data[1] +=1

    # Publisher function
    def pub_counter(self):
        self.pub.publish(self.lr_counter)


################       
# --- MAIN --- #
################

def main():

    rospy.init_node('encoders')
    enc = encoders()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        enc.pub_counter()
        rate.sleep()

if (__name__=='__main__'):
    main()