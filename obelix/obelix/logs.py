#!/usr/bin/env python3

import rospy
import pandas as pd
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque
import asyncio


class Logs:

    def __init__(self):
        self.log = pd.DataFrame()
        self.log['left_vel_set_point'] = 0.0
        self.log['left_vel'] = 0.0
        self.log['right_vel_set_point'] = 0.0
        self.log['right_vel'] = 0.0
        self.vel_list = []
        self.sub = rospy.Subscriber('/base/velocidades', Float32MultiArray, self.velocidades_callback)


    def velocidades_callback(self, msg):
        self.vel_list=msg.data
        print(self.vel_list)
        self.newrow = pd.DataFrame(self.vel_list)
        self.log = pd.concat([self.log,self.newrow])




def main():
    rospy.init_node('logs')
    logs = Logs()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
    logs.log.to_csv('/home/emerson/catkin_ws/src/tcc/obelix/obelix/log.csv')
    print('fim')



if __name__ == "__main__":
    main()
