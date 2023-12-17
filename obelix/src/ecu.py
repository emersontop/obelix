#!/usr/bin/python3

#publish com objeto

# cerebro

#importando o rospy biblioteca de acesso as informacoes do ROS
import rospy
#importando o padrao de mensagem que e enviado no topico
from geometry_msgs.msg import Twist
# Import the Time library
import time

#criando clas Controle
class Controle():

    def __init__(self):

        #pub da rodas
        self.pub = rospy.Publisher('radiator_Springs', Twist, queue_size=1)
        #Criando uma "objeto" do tipo Twist() mensagem
        self.msg = Twist()

        #sub do ultrassom
        self.sub_sensor = rospy.Subscriber('radiator_Springs2',Twist, self.callback2)
        #inicializando distancia_sensor1
        self.distancia_sensor1= 0

        #pub da visao
        self.pub_visao = rospy.Publisher('radiator_Springs4', Twist, queue_size=1)
        #Criando uma "objeto" do tipo Twist() mensagem
        self.msg_visao = Twist()
        #sub da visao
        self.sub_visao = rospy.Subscriber('radiator_Springs3',Twist, self.callback3)

        self.decisao_visao = 3



    def callback2(self, msg_sensor):
        #pega o valor do sensor ultrassonico
        self.distancia_sensor1 = msg_sensor.linear.x
        #print("Distancia callback", self.distancia_sensor1)

    def callback3(self, msg_visao):
        self.decisao_visao = msg_visao.linear.x

    def comando(self,caso):
        #metodo enviar o comando para as rodas
        self.msg.linear.x = caso

    def para(self):
        self.comando(0)
        #robo para

    def frente(self):
        self.comando(1)
        #robo vai para frente

    def direita(self):
        self.comando(2)
        #robo vai para direita

    def esquerda(self):
        self.comando(3)
        #robo vai para esquerda


## main
rospy.init_node('publ_usuario_obj')

ps1 = Controle()

#frequencia de entrega de informacao ptimize
rate = rospy.Rate(10) #10hz
#enquanto o ros nao fechar


while not rospy.is_shutdown():

    if (ps1.distancia_sensor1 < 50):
        #encontrei obstaculo o que fazer
        ps1.para()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "para")
        time.sleep(2)

        #giro de 90 graus para a direita

        ps1.direita()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "Direita 1 ")
        print(ps1.distancia_sensor1)
        time.sleep(0.4)
        distancia_direita = ps1.distancia_sensor1

        ps1.para()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "para")
        time.sleep(2)

        #giro de 180 graus para esquerda
        ps1.esquerda()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "esquerda 1")
        time.sleep(0.4)
        ps1.esquerda()

        ps1.para()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "para")
        time.sleep(1.5)

        ps1.esquerda()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "esquerda 2")
        time.sleep(0.4)
        ps1.esquerda()
        distancia_esquerda = ps1.distancia_sensor1

        ps1.para()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "para")
        time.sleep(1.5)

        ps1.direita()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "Direita 2")
        print(ps1.distancia_sensor1)
        time.sleep(0.4)

        ps1.para()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "para")
        time.sleep(1.5)

        # ja tem os valores das distancias dos extremos (direita e esquerda)

        if(distancia_direita > distancia_esquerda and distancia_direita > 50):
            ps1.direita()
            ps1.pub.publish(ps1.msg)
            print(ps1.msg.linear.x, "Direita 3 ")
            print(ps1.distancia_sensor1)
            time.sleep(0.4)

            ps1.para()
            ps1.pub.publish(ps1.msg)
            print(ps1.decisao_visao, "para")
            time.sleep(2)

        elif (distancia_esquerda > distancia_direita and distancia_esquerda > 50):
            ps1.esquerda()
            ps1.pub.publish(ps1.msg)
            print(ps1.msg.linear.x, "esquerda 3 ")
            time.sleep(0.4)
            ps1.esquerda()

            ps1.para()
            ps1.pub.publish(ps1.msg)
            print(ps1.decisao_visao, "para")
            time.sleep(1.5)

        else:
            ps1.msg_visao.linear.x = 1
            print("Solicitei a acao da visao", ps1.msg_visao.linear.x)
            ps1.pub_visao.publish(ps1.msg_visao)
            time.sleep(2)
            ps1.decisao_visao = 3
            print(ps1.decisao_visao)
            while(ps1.decisao_visao == 3):
                print("Esperando resposta da visao")

            if ps1.decisao_visao == 1:
                ps1.msg_visao.linear.x = 0
                print("Solicitei a acao da visao PARAR", ps1.msg_visao.linear.x)
                ps1.pub_visao.publish(ps1.msg_visao)

                ps1.direita()
                ps1.pub.publish(ps1.msg)
                print(ps1.msg_visao.linear.x, "decisão da visaõ direita! ")
                time.sleep(0.4)
                ps1.decisao_visao = 3

            elif ps1.decisao_visao == 0:
                ps1.msg_visao.linear.x = 0
                print("Solicitei a acao da visao PARAR", ps1.msg_visao.linear.x)
                ps1.pub_visao.publish(ps1.msg_visao)

                ps1.esquerda()
                ps1.pub.publish(ps1.msg)
                print(ps1.msg_visao.linear.x, "decisão da visaõ esquerda! ")
                time.sleep(0.4)
                ps1.decisao_visao = 3


    else:
        #indo para frente ate achar um obstaculo
        ps1.frente()
        ps1.pub.publish(ps1.msg)
        print(ps1.msg.linear.x, "Frente")
        print(ps1.distancia_sensor1)

    #controla a inatividade
    rate.sleep()
