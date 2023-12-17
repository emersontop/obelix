#!/usr/bin/env python3

# To import the RPi.GPIO module
import RPi.GPIO as GPIO

# Set up the board layout
GPIO.setmode(GPIO.BCM)

# Set up pin 25 as output
GPIO.setup(25,GPIO.OUT)

# Set up pin 25 as PWM with a 1000 Hz frequency
pwm25=GPIO.PWM(25,1000)

# Start with a duty cycle of 50%
pwm25.start(50)

# Change the frequency
pwm25.ChangeFrequency(500)

# Change the duty cycle
pwm25.ChangeDutyCycle(100)

# Stop
pwm25.stop()

# Cleanup at the end of the program
GPIO.cleanup()
