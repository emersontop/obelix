#!/usr/bin/env python3

# To import the RPi.GPIO module
import RPi.GPIO as GPIO

# Set up the board layout
GPIO.setmode(GPIO.BCM)

# Set up pin 24 as output
GPIO.setup(24,GPIO.OUT)

# Turn OFF pin 24
GPIO.output(24,GPIO.LOW)

# Turn ON pin 24
GPIO.output(24,GPIO.HIGH)

# Cleanup at the end of the program
GPIO.cleanup()
