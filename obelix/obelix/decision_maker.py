#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: decison_maker
#Descrição: configura a maquina de estados geral

import os
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

TEMPO_ROTACAO_ESQUERDA = 2.5
TEMPO_ROTACAO_DIREITA = 2.5
TEMPO_MOVER_PARA_TRAS = 2

class Decision_maker():
    
    def __init__(self):
        # estado inicial
        self.estado = self.alternador_de_estados('iniciar')

        # mensagem_resposta
        self.eventos = {
            'resposta_lidar': 2.1,
            'resposta_camera': str(),
            'direita': 'vermelho',
            'esquerda': 'vermelho',
            'trazeira': 'vermelho'
        }

        # mensagens_enviadas
        self.mensagem_base = Twist()
        self.mensagem_camera = Int32()
        self.mensagem_servo = Int32()

        # pub
        #self.pub_camera = rospy.Publisher('camera', Int32, queue_size=1)
        self.pub_servo = rospy.Publisher('servo', Int32, queue_size=1)
        self.pub_base = rospy.Publisher('/base/cmd', Twist, queue_size=1)

        # sub
        #self.camera = rospy.Subscriber('camera_resposta', Int32, self.callback_camera)
        self.lidar = rospy.Subscriber('lidar_resposta',Float64,self.callback_lidar)

        # cliente
        self.cliente_camera = rospy.ServiceProxy('capture_and_analyze', Trigger)

    #CALLBACKS

    def callback_camera(self, msg_camera):
        '''Recebe a mensagem da camera'''

        self.eventos['resposta_camera'] = msg_camera.data
        #print("resposta_camera: ", self.resposta_camera)

    def callback_lidar(self, msg_lidar):
        '''Recebe a mensagem do lidar'''

        self.eventos['resposta_lidar'] = msg_lidar.data
        #print("resposta_lidar: ", self.resposta_lidar)


    # Maquina de estados

    def alternador_de_estados(self, estado):
        '''todos os estados'''
        self.estados = {
            'iniciar': self.setup_robo,
            'em_frente': self.frente,
            'obstaculo_detectado': self.obstaculo_detectado,
            'procurar_saidas': self.procurar_saidas,
            'evitar_obstaculo': self.evitar_obstaculo
        }
        return self.estados[estado]
    
    def em_evento(self, eventos):
        '''em evento'''
        self.estado = self.estado(self.eventos)
    
    # Estados 

    def setup_robo(self, eventos):
        '''Realiza a rotina para iniciar o robo'''
        print("Inicinando robo em ... ")
        time.sleep(2)
        print('3 ...')
        time.sleep(2)
        print('2 ...')
        time.sleep(2)
        print('1 ...')
        return self.alternador_de_estados('em_frente')
    
    def frente(self, eventos):
        '''ir em frente'''
        print("Vamos em frente !")

        if (eventos['resposta_lidar'] > 0.2):
            self.mover_robo_frente()
            return self.alternador_de_estados('em_frente')
        return self.alternador_de_estados('obstaculo_detectado')

    def obstaculo_detectado(self, eventos):
        print('Obstaculo detectado')
        print('Esperando motor para...')
        self.parar()
        time.sleep(3)
        # if (eventos['resposta_lidar'] > 0.2):
        #     return self.alternador_de_estados('em_frente')
        return self.alternador_de_estados('procurar_saidas')     

    def procurar_saidas(self, eventos):
        self.rotacionar_servo_direita()
        print('procurando na direita')
        eventos['direita'] = self.solicita_analise_camera()
        print(f"lado direito {eventos['direita']}")
        self.rotacionar_servo_esquerda()
        print('procurando na esquerda')
        eventos['esquerda'] = self.solicita_analise_camera()
        print(f"lado esquerda {eventos['esquerda']}")
        self.rotacionar_servo_frente()
        time.sleep(2)
        return self.alternador_de_estados('evitar_obstaculo')            

    def evitar_obstaculo(self, eventos):
        

        if (eventos['direita']=='vermelho')&(eventos['esquerda']=='vermelho'):
            print('desviar de obstaculo indo para tras')
            self.mover_para_tras()
            self.rotacionar_robo_direita()
            return self.alternador_de_estados('em_frente')
        elif (eventos['direita']=='verde')&(eventos['esquerda']=='vermelho'):
            print('desviar de obstaculo pela direita')
            self.rotacionar_robo_direita()
            return self.alternador_de_estados('em_frente')
        elif (eventos['direita']=='vermelho')&(eventos['esquerda']=='verde'):
            print('desviar de obstaculo pela esquerda')
            self.rotacionar_robo_esquerda()
            return self.alternador_de_estados('em_frente')
        elif (eventos['direita']=='verde')&(eventos['esquerda']=='verde'):
            print('desviar de obstaculo pela direita')
            self.rotacionar_robo_direita()
            return self.alternador_de_estados('em_frente')

    # Metodos gerais

    def solicita_analise_camera(self):
        rospy.wait_for_service('capture_and_analyze')
        try:
            response = self.cliente_camera()
            self.eventos['resposta_camera'] = str(response.message)
            return self.eventos['resposta_camera']
        except rospy.ServiceException as e :
            rospy.logerr(f'Erro ao chamar servico: {e}')
        return 4

    def velocidade_desejada(self, linear=0.0, angular=0.0):
        '''setar velocidade desejada'''

        self.mensagem_base.linear.x = linear
        self.mensagem_base.angular.z = angular
        self.pub_base.publish(self.mensagem_base)

    def posicao_desejada_servo(self, angulo=0):
        '''Muda a posicao do servo para a desejada'''

        self.mensagem_servo.data = angulo
        self.pub_servo.publish(self.mensagem_servo)

    def mover_robo_frente(self):
        angular = 0.0
        linear = 0.3
        self.velocidade_desejada(linear,angular)

    def rotacionar_robo_esquerda(self):
        '''Rotacionar o robo para esquerda'''

        angular = 1.0
        linear = 0.0
        self.velocidade_desejada(linear,angular)
        time.sleep(TEMPO_ROTACAO_ESQUERDA)
        self.velocidade_desejada(0,0)

    def rotacionar_robo_direita(self):
        '''Rotacionar o robo para direita'''

        angular = -1.0
        linear = 0.0
        self.velocidade_desejada(linear,angular)
        time.sleep(TEMPO_ROTACAO_DIREITA)
        self.velocidade_desejada(0,0)

    def parar(self):
        '''parar'''

        angular = 0.0
        linear = 0.0
        self.velocidade_desejada(linear,angular)

    def mover_para_tras(self):
        angular = 0.0
        linear = -0.3
        self.velocidade_desejada(linear,angular)
        time.sleep(TEMPO_MOVER_PARA_TRAS)
        self.velocidade_desejada(0,0)

    def rotacionar_servo_esquerda(self):
        '''Rotacionar servo para esquerda'''

        self.posicao_desejada_servo(90)
    
    def rotacionar_servo_direita(self):
        '''Rotacionar servo para direita'''

        self.posicao_desejada_servo(-90)

    def rotacionar_servo_frente(self):
        '''Rotacionar servo para direita'''

        self.posicao_desejada_servo(0)

def main():
    rospy.init_node('decision_maker')
    decision_maker = Decision_maker()
    rospy.loginfo('Inicio no Decision_maker')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        decision_maker.em_evento(decision_maker.eventos)
        rate.sleep()

if (__name__ == '__main__'):
    main()