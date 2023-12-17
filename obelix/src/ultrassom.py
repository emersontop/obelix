#!/usr/bin/python3

# pub

# CamJam EduKit 3 - Robotics
# Worksheet 6 - Measuring Distance

import time # Import the Time library
from gpiozero import DistanceSensor # Import GPIO Zero Library

#importando o rospy biblioteca de acesso as informacoes do ROS
import rospy

#importando o tipo de mensagemsensor0
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Distancia():
#1 = msg_sensor.linear.x
    def __init__(self):

        #self.pub = rospy.Publisher('/scan_raw',LaserScan, queue_size=1 )
        self.pub = rospy.Publisher('radiator_Springs2',Twist, queue_size=1 )

        # Define GPIO pins to use on the Pi
        self.pintrigger = 17
        self.pinecho = 18

        #criar o obj distancia
        self.sensor = DistanceSensor(echo=self.pinecho, trigger=self.pintrigger)

        #criar  o padrao de mensagem
        self.msg_sensor = Twist()

    def comando(self):
        #metodo para enviar a mensagem.

        #passando uma informacao para atributo do objeto LaserScan
        #self.msg_sensor.ranges.append(self.sensor.distance)
        self.msg_sensor.linear.x = self.sensor.distance*100
        #print("Distance: %.1f cm" % (self.msg_sensor.ranges * 100))
        print("Distance: %.1f cm" % (self.msg_sensor.linear.x))

## main
rospy.init_node('ultrassom')

sensor1 = Distancia()

#frequencia de entrega de informacao ptimize
rate = rospy.Rate(10) #10hz

#enquanto o ros nao fechar
while not rospy.is_shutdown():
    sensor1.comando()
    sensor1.pub.publish(sensor1.msg_sensor)
    
    #controla a inatividade
    rate.sleep()
