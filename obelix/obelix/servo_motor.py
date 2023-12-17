#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: servo_motor
#Descrição: movimentar o servo motor

from gpiozero import AngularServo
from time import sleep
import rospy
from std_msgs.msg import Int32

PIN_SERVO = 14

class Servo(object):
    
    def __init__(self):
        #setup

        self.servo = AngularServo(PIN_SERVO, min_pulse_width=0.0004, max_pulse_width=0.0025)

        self.angulo_servo = rospy.Subscriber('servo', Int32, self.callback_angulo)

    def callback_angulo(self,msg_angulo):
        '''Altera o angulo do servo'''

        self.servo.angle = msg_angulo.data


def main():
    rospy.init_node('servo_motor')
    servo = Servo()
    rospy.loginfo('Inicio no locomocao')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if (__name__ == '__main__'):
    main()