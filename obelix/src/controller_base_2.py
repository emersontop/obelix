#!/usr/bin/env python3

# Node information:
# Name: controller_base
# Description: Kinematics and base control

# importing libraries
import RPi.GPIO as GPIO        
import time
import math
#from apscheduler.schedulers.background import BackgroundScheduler

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class controller:

    def __init__(self):

        ############
        # Subscriber
        rospy.Subscriber('/base/encoders', Float32MultiArray, self.cb_encoders, queue_size=1)
        rospy.Subscriber('/base/cmd', Twist, self.cb_velocity, queue_size=1)

        # publisher
        self.pub = rospy.Publisher('/base/velocidades', Float32MultiArray, queue_size=1)
        self.velocidades = Float32MultiArray()
        self.velocidades.data = [0]*4

        ####################
        # Robot's parameters
        self.interWheels = 0.2 ### to be checked
        self.reductionRatio = 46
        self.nbPoles = 20
        self.wheelDiameter = 0.067
        self.pulseToRad = 2*math.pi / (self.reductionRatio*self.nbPoles)
        self.v = 0.10 # linear velocity
        self.w = 0.0 # angular velocity

        ########################
        # Controller's paramters
        self.ts = 0.1 # sampling time
        self.kp = 4.5
        self.ki = 1.1
        self.kd = 0.2
        self.leftInt = 0
        self.rightInt = 0
        self.refLeftVelocity = 0.3
        self.refRightVelocity = 0.3
        self.maxAngVelocity = 130/60*math.pi*2
        self.initEncoder = False
        
        ########################################
        # Variables to compute the current speed
        self.timeInterval = 0.1
        self.previousLeft = 0.0
        self.previousRight = 0.0
        self.leftVelocity = 0.0
        self.rightVelocity = 0.0

        #######################
        # Setup the pin numbers

        # For the right motor
        self.mr_fwd = 23    #in1
        self.mr_bck = 24    #in2
        self.mr_en = 25     #ina

        # For the left motor
        self.ml_fwd = 22    #in3
        self.ml_bck = 27    #in4
        self.ml_en = 17     #inb

        ###################
        # Configure the pin

        GPIO.setmode(GPIO.BCM)

        # Right motor
        GPIO.setup(self.mr_fwd,GPIO.OUT)
        GPIO.setup(self.mr_bck,GPIO.OUT)
        GPIO.setup(self.mr_en,GPIO.OUT)
        GPIO.output(self.mr_fwd,GPIO.LOW)
        GPIO.output(self.mr_bck,GPIO.LOW)
        self.mr=GPIO.PWM(self.mr_en,1000)
        self.mr.start(0) #46% -> 60 RPM

        # Left motor
        GPIO.setup(self.ml_fwd,GPIO.OUT)
        GPIO.setup(self.ml_bck,GPIO.OUT)
        GPIO.setup(self.ml_en,GPIO.OUT)
        GPIO.output(self.ml_fwd,GPIO.LOW)
        GPIO.output(self.ml_bck,GPIO.LOW)
        self.ml=GPIO.PWM(self.ml_en,1000)
        self.ml.start(0)

    def __del__(self):

        self.ml.ChangeDutyCycle(0)
        self.mr.ChangeDutyCycle(0)

        GPIO.output(self.mr_fwd,GPIO.LOW)
        GPIO.output(self.mr_bck,GPIO.LOW)
        GPIO.output(self.ml_fwd,GPIO.LOW)
        GPIO.output(self.ml_bck,GPIO.LOW)

        GPIO.cleanup()

    # Sign function
    def my_sign(self, x):
        return (x > 0) - (x < 0)

    # Callback encoder
    def cb_encoders(self, msg):
        '''
            Le o pulso dos encoders, 
            atualiza os pulsos de cada motor
            Calcula a velocidade de cada motor
            Atualiza o valor da velocidade       
        '''
        # Save the current number of counts
        left = float(msg.data[0])
        right = float(msg.data[1])

        # Init the previous values to deal with non zero initial encoders
        if not self.initEncoder:
            self.previousLeft = left
            self.previousRight = right
            self.initEncoder = True

        # Compute the motor velocity
        self.leftVelocity = (left - self.previousLeft) / self.timeInterval * self.pulseToRad
        self.rightVelocity = (right - self.previousRight) / self.timeInterval * self.pulseToRad 
        
        # Update the previous number of counts
        self.previousLeft = left
        self.previousRight = right 

    # Callback velocities
    def cb_velocity(self, msg):
        '''Recebe o valor desejado de velocidade'''

        # Update the velocities
        self.v = msg.linear.x
        self.w = msg.angular.z

    # PID controller
    def pid(self):

        # Convert the base velocities into wheels velocities
        # converte as velocidades desejadas em velocidade da roda
        delta = self.interWheels*self.w / (self.wheelDiameter/2)
        l_vel = self.v/(self.wheelDiameter/2) - delta/2
        r_vel = self.v/(self.wheelDiameter/2) + delta/2

        # Adjustment based on feedbacks
        leftErr = l_vel - self.leftVelocity
        self.leftInt += leftErr*self.ts
        leftDer = leftErr / self.ts
        #l_vel_send += self.kp*leftErr + self.kd*leftDer + self.ki*self.leftInt
        l_vel_send = l_vel+ self.kp*leftErr + self.kd*leftDer + self.ki*self.leftInt

        rightErr = r_vel - self.rightVelocity
        self.rightInt += rightErr*self.ts
        rightDer = rightErr / self.ts
        #r_vel_send += self.kp*rightErr + self.kd*rightDer + self.ki*self.rightInt
        r_vel_send = r_vel + self.kp*rightErr + self.kd*rightDer + self.ki*self.rightInt

        # Adjust velocties accordind to motors bounds  
        if abs(l_vel_send) > self.maxAngVelocity:
            ratio = self.maxAngVelocity/abs(l_vel_send)
            l_vel_send = self.my_sign(l_vel_send)*self.maxAngVelocity
            r_vel_send *= ratio

        if abs(r_vel_send) > self.maxAngVelocity:
            ratio = self.maxAngVelocity/abs(r_vel_send)
            r_vel_send = self.my_sign(r_vel_send)*self.maxAngVelocity
            l_vel_send *= ratio

        # Detect the wheels direction
        if l_vel_send >= 0:
            l_forward = GPIO.HIGH
            l_backward = GPIO.LOW
        else:     
            l_forward = GPIO.LOW
            l_backward = GPIO.HIGH

        if r_vel_send >= 0:
            r_forward = GPIO.HIGH
            r_backward = GPIO.LOW
        else:     
            r_forward = GPIO.LOW
            r_backward = GPIO.HIGH    

        # Convert wheels velocity into duty cycle
        l_dc = math.floor(abs(l_vel_send) / self.maxAngVelocity*100)
        r_dc = math.floor(abs(r_vel_send) / self.maxAngVelocity*100)

        self.velocidades.data[0] = l_vel
        self.velocidades.data[1] = self.leftVelocity
        self.velocidades.data[2] = r_vel
        self.velocidades.data[3] = self.rightVelocity
        #print("Desired left: {} - Measured left : {} - Send left: {}".format(l_vel,self.leftVelocity,l_vel_send))
        #print("Desired right: {} - Measured right : {}- Send right: {}".format(r_vel,self.rightVelocity,r_vel_send))

        # Update the duty cycle
        self.ml.ChangeDutyCycle(l_dc)
        self.mr.ChangeDutyCycle(r_dc)

        # Update the motors direction
        GPIO.output(self.ml_fwd,l_forward)
        GPIO.output(self.ml_bck,l_backward)
        GPIO.output(self.mr_fwd,r_forward)
        GPIO.output(self.mr_bck,r_backward)

    # Publisher function
    def pub_velocidades(self):
        self.pub.publish(self.velocidades)
        

################       
# --- MAIN --- #
################

rospy.init_node('controller_2')
cont = controller()

# rospy.spin()

rate = rospy.Rate(1/cont.ts)

while not rospy.is_shutdown():
    cont.pid()
    cont.pub_velocidades()
    rate.sleep()