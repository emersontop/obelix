#!/usr/bin/python3

import rospy        ##biblioteca para ros
from std_msgs.msg import Int32       #biblioteca de mensagens ros

class NodeComunicacao():
    """Classe ros para genrenciar as mensage"""
    def __init__(self):
        """Instnacia o objeto"""
        self.direcao = Int32()              #instanciando a variável que envia a direção

        self.pubDirecao = rospy.Publisher('tpclocomocao',Int32,queue_size=1)    #instancia o pub


class ControleRemoto():
    """Controle remoto para robo"""
    def __init__(self):
        """Instancia o objeto controle remoto"""
        self.direcao = 0

    def controle(sefl,comando):
        """Le o comando do teclado e traduz direção"""
        if(comando == "w"):
            comando=0
            return 1
        elif(comando == "s"):
            comando=0
            return 2
        else:
            comando=0
            return 0
        
def main():
    rospy.init_node("controle")
    rospy.loginfo("O node controle remoto foi iniciado")

    nodeComunicao = NodeComunicacao()       #Instancia a comunicação

    controleRemoto = ControleRemoto()

    while(not rospy.is_shutdown()):
        try:
            comando = input()
            direcao = controleRemoto.controle(comando)
            nodeComunicao.pubDirecao.publish(direcao)

        except KeyboardInterrupt():
            print("Fim")

if(__name__ == "__main__"):
    main()