
import RPi.GPIO as GPIO     #biblioteca para a raspybarry  
from time import sleep      #biblioteca para trabalhar com o tempo

#Variaveis globais

contador = 0

#definicao de pinos

PIN_AMARELO = 16    #fio amarelo, encoder A phase > pino GPIO 6
#PIN_VERDE = 5      #fio verde, encoder B phase > pino GPIO 5

def conta(channel):
    '''Funcaoo para contar'''
    global contador
    contador +=1
    #print("leitura do encoder- contador: ",contador)

#Setup
GPIO.setmode(GPIO.BCM)      # Numeracao nomes GPIO 

GPIO.setup(PIN_AMARELO,GPIO.IN)

GPIO.add_event_detect(PIN_AMARELO,GPIO.RISING,callback=conta)

try:

    while True:
        
        sleep(1)
        print("pulsos/seg: ", contador)
        contador = 0

except KeyboardInterrupt:

    GPIO.cleanup()