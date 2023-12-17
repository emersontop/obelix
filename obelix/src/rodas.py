#!/usr/bin/python3

#controle das rodas

#importando o rospy biblioteca de acesso as informacoes do ROS
import rospy

#importando o padrao de mensagem que e enviado no topico
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Import the Time library
import time

# Import the CamJam GPIO Zero Library
from gpiozero import CamJamKitRobot

class Wheel_controller():

    def __init__(self):
        self.sub = rospy.Subscriber('radiator_Springs',Twist, self.callback)
        #sub para o sensor som 1
        #self.sub_sensor = rospy.Subscriber('/scan_raw',LaserScan, self.callback2)


        self.speed = 0
        self.robot = CamJamKitRobot()
        self.direcao = 0
        self.time = 0
        self.caso = 0


    def callback(self, msg):
        #Calback da ECU
        self.caso = msg.linear.x

    def para(self):
        motorspeed = 0
        compensador = 1.3
        motorforward = (motorspeed, (compensador*motorspeed))
        self.robot.value = motorforward

    def mover_frente(self):
        motorspeed = 0.3
        compensador = 1.3
        motorforward = (motorspeed, (compensador*motorspeed))
        self.robot.value = motorforward

    def direita(self):
        motorspeed = 0.3
        compensador = 1.3
        motorright = (motorspeed, -compensador*motorspeed)
        self.robot.value = motorright
        time.sleep(0.4)

    def esquerda(self):
        motorspeed = 0.3
        compensador = 1.3
        motorleft = (-motorspeed, compensador*motorspeed)
        self.robot.value = motorleft
        time.sleep(0.4)


#iniciando o no. Dentro no () vai o nome do no
rospy.init_node("sub_robo")

obelix = Wheel_controller()

while not rospy.is_shutdown():

    if obelix.caso == 0:
        obelix.para()
    if obelix.caso == 1:
        obelix.mover_frente()
    if obelix.caso == 2:
        obelix.direita()
    if obelix.caso == 3:
        obelix.esquerda()

# rospy.spin()
