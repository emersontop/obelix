#!/usr/bin/env python3

# To import the RPi.GPIO module
import RPi.GPIO as GPIO

# Definition of the callback function for event detection
def cb_function(channel):
    print("Hello World")


GPIO.setmode(GPIO.BCM)

# Set up pin 10 as input
GPIO.setup(10,GPIO.IN)

# Add event detection for pin 10
GPIO.add_event_detect(10,GPIO.RISING,callback=cb_function)

# Remove event detection for pin 10
GPIO.remove_event_detect(channel)
