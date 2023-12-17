#!/usr/bin/env python3

# Node information:
# Name: Lidar
# Description: captures data from the lidar and sends the average distance

# importing libraries
import os
import sys
import time
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import ydlidar
import rospy
from std_msgs.msg import Float64

RMAX = 32.0

class Lidar():

    def __init__(self):
        #self.setup_plot_lidar()
        self.setup_lidar()
        
        self.lidar_resposta = Float64()
        #pub
        self.pub = rospy.Publisher('lidar_resposta', Float64, queue_size=1)

    
    def setup_plot_lidar(self):
        '''setup plot lidar'''

        self.fig = plt.figure()
        self.fig.canvas.set_window_title('YDLidar LIDAR Monitor')
        self.theta_offset_deg = 45
        self.theta_offset_rad = np.radians(self.theta_offset_deg)
        self.lidar_polar = plt.subplot(polar=True, theta_offset=self.theta_offset_rad)
        self.lidar_polar.set_rmax(RMAX)
        self.lidar_polar.autoscale_view(True,True,True)
        self.lidar_polar.grid(True)

    def setup_lidar(self):
        self.ports = ydlidar.lidarPortList()    
        self.port = "/dev/ydlidar"
        for key, value in self.ports.items():
            self.port = value
    
        self.laser = ydlidar.CYdLidar();
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port);
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 9);
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, -90.0);
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 2.0);
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08);
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
        self.scan = ydlidar.LaserScan()

    def animate(self, num):
    
        r = self.laser.doProcessSimple(self.scan);
        if r:
            angle = []
            ran = []
            intensity = []
            for point in self.scan.points:
                angle.append(point.angle);
                ran.append(point.range);
                intensity.append(point.intensity);
            range_sem_zeros = np.array(ran)[np.array(ran)!=0]
            #if len(range_sem_zeros) > 0:
                #print("A menor distancia eh: ", np.mean(range_sem_zeros))
            self.lidar_polar.clear()
            self.lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)

    def visualzar_animacao(self):
        ret = self.laser.initialize();
        if ret:
            ret = self.laser.turnOn();
            if ret:
                ani = animation.FuncAnimation(self.fig, self.animate, interval=50)
                plt.show()
            self.laser.turnOff();
        self.laser.disconnecting();
        plt.close();

    def iniciar_lidar(self):
        ret = self.laser.initialize();
        if ret:
            ret = self.laser.turnOn();

    def finalizar_lidar(self):

        self.laser.turnOff();
        self.laser.disconnecting();

    def ler_distancia(self):
        
        r = self.laser.doProcessSimple(self.scan);
        if r:
            angle = []
            ran = []
            intensity = []
            for point in self.scan.points:
                angle.append(point.angle);
                ran.append(point.range);
                intensity.append(point.intensity);
            
            range_sem_zeros = np.array(ran)[np.array(ran)!=0]
            
            if len(range_sem_zeros) > 0:
                self.lidar_resposta.data = np.mean(range_sem_zeros)
                #print("A menor distancia eh: ", self.lidar_resposta.data)
                self.pub.publish(self.lidar_resposta)

def main():
    rospy.init_node('lidar')
    lidar = Lidar()
    rospy.loginfo('Inicio no lidar')
    
    lidar.iniciar_lidar()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        lidar.ler_distancia()
        rate.sleep()
    
    lidar.finalizar_lidar()

if (__name__ == '__main__'):
    main()