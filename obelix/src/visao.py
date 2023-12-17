#!/usr/bin/python3

## Controle dos servos
#controle da camera e analise das imagens

#sub da ECU
#pub da ecu

#importando o rospy biblioteca de acesso as informacoes do ROS
import rospy

from gpiozero import Servo
import time
import cv2
from geometry_msgs.msg import Twist


class Visao():

    def __init__(self):

        #pud da ecu
        self.pub = rospy.Publisher('radiator_Springs3', Twist, queue_size=1)
        #Criando uma "objeto" do tipo Twist() mensagem
        self.msg_visao = Twist()

        #sub da ecu
        self.sub_ecu = rospy.Subscriber('radiator_Springs4',Twist, self.callback4)

        self.servo = Servo(25,0, 0.5/1000,2.6/1000,20/1000)
        self.on_off = 0

    def callback4(self,msg_visao):
        self.on_off= msg_visao.linear.x

##MAIN

rospy.init_node('Visao')

foto = Visao()

#frequencia de entrega de informacao ptimize
rate = rospy.Rate(10) #10hz
#enquanto o ros nao fechar

while not rospy.is_shutdown():

    if(foto.on_off==1):
        ##esquerda
        foto.servo.max()
        time.sleep(1)

        # open camera
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

        # set dimensions
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

        # take frame
        ret, frame = cap.read()
        dim = (480, 320)
        frame_resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        # write frame to file
        # cv2.imwrite('image.jpg', frame)
        # #cv2.imwrite('imageBlue.jpg', frame[:,:,0])
        # cv2.imwrite('imageGreen.jpg', frame[:,:,1])
        # cv2.imwrite('imageVermelho.jpg', frame[:,:,2])

        pixelGreen = 0
        pixelRed = 0

        #Analisando a imagem
        for i in range(479):
            for j in range(319):
                #print("coluna",j, "Linha",i)
                if (frame_resized[j][i][1]<50):
                    pixelRed = pixelRed + 1
                if (frame_resized[j][i][2]<50):

                    pixelGreen = pixelGreen + 1

        if (pixelRed>pixelGreen):
            cor_esquerda = 0 #vermelho
            print("Cor Esquerda vermelha",cor_esquerda)
            print("Pixels vermelhos",pixelRed)
            print("Pixels verdes",pixelGreen)

        else:
            cor_esquerda = 1 #verde
            print("Cor Esquerda verde",cor_esquerda)
            print("Pixels vermelhos",pixelRed)
            print("Pixels verdes",pixelGreen)

        #print("A cor é", cor, "a porcentagem de verde é:", 100*(pixelGreen/(480*320)), "a porcentágem de vermelho é", 100*(pixelRed/(480*320)))

        # release camera
        cap.release()
        time.sleep(1)

        foto.servo.mid()
        time.sleep(1)

        ##Esquerda
        foto.servo.min()
        time.sleep(1)

        # open camera
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

        # set dimensions
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

        # take frame
        ret, frame = cap.read()
        dim = (480, 320)
        frame_resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        # write frame to file
        # cv2.imwrite('image.jpg', frame)
        # #cv2.imwrite('imageBlue.jpg', frame[:,:,0])
        # cv2.imwrite('imageGreen.jpg', frame[:,:,1])
        # cv2.imwrite('imageVermelho.jpg', frame[:,:,2])

        pixelGreen = 0
        pixelRed = 0

        #Analisando a imagem
        for i in range(479):
            for j in range(319):
                #print("coluna",j, "Linha",i)
                if (frame_resized[j][i][1]<50):
                    #na matriz verde se for menor marca vermelho
                    pixelRed = pixelRed + 1
                if (frame_resized[j][i][2]<50):
                    #na matriz vermelha se for menor marca verde
                    pixelGreen = pixelGreen + 1

        if (pixelRed>pixelGreen):
            cor_direita = 0 #vermelho
            print("Cor direita vermelho",cor_direita)
            print("Pixels vermelhos",pixelRed)
            print("Pixels verdes",pixelGreen)
        else:
            cor_direita = 1 #verde
            print("Cor direita verde",cor_direita)
            print("Pixels vermelhos",pixelRed)
            print("Pixels verdes",pixelGreen)

        # release camera
        cap.release()
        time.sleep(1)

        foto.servo.mid()
        time.sleep(1)

        if (cor_direita ==1):
            foto.msg_visao.linear.x = 1
            foto.pub.publish(foto.msg_visao)
        else:
            foto.msg_visao.linear.x = 0
            foto.pub.publish(foto.msg_visao)

        foto.on_off=0

    #controla a inatividade
    rate.sleep()
