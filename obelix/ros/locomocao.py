#!/usr/bin/python3

import rospy        ##biblioteca para ros
from std_msgs.msg import Int32       #biblioteca de mensagens ros
import RPi.GPIO as GPIO     #biblioteca para a raspybarry  
from time import sleep      #biblioteca para trabalhar com o tempo

#variaveis globais

globalDirecao = 0          #recebe o comando da direção

class Nodelocomocao():
    """classe node ros da locomoção, responsável por gerenciar as mensagens do sistema da locomoção"""
    def __init__ (self):
        self.direcao = Int32()          #Instancia a variável da direção

        self.subdirecao = rospy.Subscriber('tpclocomocao', Int32, self.callbacklocomocao)

    def callbacklocomocao(self,msgLocomocao):
        global globalDirecao
        self.direcao = msgLocomocao.data            #usa o .data para pegar o valor
        globalDirecao = self.direcao


class Locomocao():
    def __init__(self):
        self.in1 = 23
        self.in2 = 24
        self.in3 = 22
        self.in4 = 27
        self.ena = 25
        self.enb = 17

        self.estado = Int32()       #controla o estado do robo
        self.speedRodas = Int32()   #Controla a velocidade do robo

        #Definição dos pinos
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.in3,GPIO.OUT)
        GPIO.setup(self.in4,GPIO.OUT)
        GPIO.setup(self.ena,GPIO.OUT)
        GPIO.setup(self.enb,GPIO.OUT)

        #Configuração inicial dos pinos
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW)

        self.motor1=GPIO.PWM(self.ena,1000)
        self.motor2=GPIO.PWM(self.enb,1000)

        self.motor1.start(25)
        self.motor2.start(25)

        print("\n")
        print("Iniciando o sistema de locomoção")
        print("Comandos :")
        print("[r-run] [s-stop] [f-forward] [b-backward] [l-low] [m-medium] [h-high] [e-exit]")
        print("\n")

    def speed(self, speed=100):
        """Define a velocidade do robô"""
        #por enquanto a velocidade esta setada para 50
        print("Velocidade das rodas configuradas para: ",speed)
        self.speedRodas = speed
        self.motor1.ChangeDutyCycle(self.speedRodas)
        self.motor2.ChangeDutyCycle(self.speedRodas)

    def forward(self):
        """Move o robo para frente"""
        print("forward")
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.HIGH)
        GPIO.output(self.in4,GPIO.LOW)
        sleep(3)

    def backward(self):
        """Move o robo para traz"""
        print("backward")
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.HIGH)
        sleep(3)

    def stop(self):
        """Para o robo"""
        print("stop")
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW)
class Locomocao():
    def __init__(self):
        #definição de pinos motor e ponte h

        self.in1 = 23
        self.in2 = 24
        self.in3 = 22
        self.in4 = 27
        self.ena = 25
        self.enb = 17

        #definição pinos encoder

        self.estado = 0   #controla o estado do robo
        self.speedRodas = 100  #Controla a velocidade do robo

        #Definição dos pinos
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.in3,GPIO.OUT)
        GPIO.setup(self.in4,GPIO.OUT)
        GPIO.setup(self.ena,GPIO.OUT)
        GPIO.setup(self.enb,GPIO.OUT)

        #Configuração inicial dos pinos
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW)

        self.motor1=GPIO.PWM(self.ena,1000)
        self.motor2=GPIO.PWM(self.enb,1000)

        self.motor1.start(25)
        self.motor2.start(25)

        print("\n")
        print("Iniciando o sistema de locomoção")
        print("Comandos :")
        print("[r-run] [s-stop] [f-forward] [b-backward] [l-low] [m-medium] [h-high] [e-exit]")
        print("\n")

    def speed(self, speed=100):
        """Define a velocidade do robô"""
        #por enquanto a velocidade esta setada para 50
        print("Velocidade das rodas configuradas para: ",speed)
        self.speedRodas = speed
        self.motor1.ChangeDutyCycle(self.speedRodas)
        self.motor2.ChangeDutyCycle(self.speedRodas)

    def forward(self):
        """Move o robo para frente"""
        print("forward")
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.HIGH)
        GPIO.output(self.in4,GPIO.LOW)
        sleep(3)

    def backward(self):
        """Move o robo para traz"""
        print("backward")
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.HIGH)
        sleep(3)

    def stop(self):
        """Para o robo"""
        print("stop")
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW)

    def controle(self, direcao):
        """Recebe a direção e chama os metodos ro robo"""
        #print("Direção: ", direcao)
        if(direcao == 1):
            self.forward()
        elif(direcao == 2):
            self.backward()
        else:
            #self.stop()
            #print("Comando não conhecido! Tente outra vez!")
            pass

def main():
    #garante o uso da variável global
    global globalDirecao

    #Setup ROS
    rospy.init_node('locomocao')                #inicia o Node
    rospy.loginfo('O node da locomoção iniciado!')

    nodelocomocao = Nodelocomocao()         #instancia o node da locomoção

    locomocao = Locomocao()                 #instancia a classe locomoção
    locomocao.speed()

    print("Teste")
    while (not rospy.is_shutdown()):
        try:
            locomocao.controle(globalDirecao)
            
        except KeyboardInterrupt():
            print("Fim")


if (__name__ == "__main__"):
    main()