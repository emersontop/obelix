#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: Camera
#Descrição: Abrir a camera zed e enviar as imagens no Topico imagens

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image   #tipo de mensagem para enviar imagem
from cv_bridge import CvBridge, CvBridgeError #converter um objeto do open cv para mensagem imagem padrão do ros

#Aqui é definido a classe desse node
class NodeCamera():

    def __init__(self):

        #Atributos ROS
        self.bridge = CvBridge()    #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.msgImagem = Image()    #guarda a mensagem do tipo imagem que sera enviada

        self.pubImagem = rospy.Publisher('TPC1Camera',Image,queue_size=1)   #guarda os parametros para o envio da imagem
        
        self.cam = cv2.VideoCapture(0)        #instanciando camera

    #entra em mode de operação
    def nodeWorking(self):
        try:
            while not rospy.is_shutdown():
                
                #Captura a imagem

                validacao, frame = self.cam.read()
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                #frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
                #frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #Converte a imagem para um formato que pode ser enviado
                self.msgImagem = self.bridge.cv2_to_imgmsg(frame,"bgr8")
                self.pubImagem.publish(self.msgImagem)
        except KeyboardInterrupt:
            self.cam.release()
            print('FIM')


def main():
    
    #Setup ROS
    rospy.init_node('camera_raspy') #inicia o Node
    rospy.loginfo('O node camera_raspy foi iniciado!')

    nodeCamera = NodeCamera() #instanciando o objeto
    nodeCamera.nodeWorking()

if __name__ == '__main__':
    main()