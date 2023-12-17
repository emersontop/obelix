#!/usr/bin/env python3

# Node information:
# Name: camera
# Description: Camera node as as service that captures and return image analysis

# importing libraries
import rospy
import cv2
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse

class Camera():
    '''Class to manage the node'''
    def __init__(self):
        self.pixelRed = 0
        self.pixelGreen = 0
        self.threshold = 50
        self.new_dimension_image = (480, 320)

        # service
        self.camera_service = rospy.Service('capture_and_analyze', Trigger, self.capture_and_analyze)


    def capture_and_analyze(self, req):
        '''Capture ande analyze image'''

        self.camera_image = cv2.VideoCapture(0)

        try:
            if self.camera_image.isOpened():
                validation, frame = self.camera_image.read()
                if validation:
                    cv2.imshow("Camera",frame)
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    cv2.imwrite('/home/emerson/catkin_ws/src/tcc/obelix/imagens/image.jpg', frame)

                    frame_resized = cv2.resize(frame, self.new_dimension_image, interpolation = cv2.INTER_AREA)
        
                    for i in range(self.new_dimension_image[0]):
                        for j in range(self.new_dimension_image[1]):
                            if (frame_resized[j][i][1]<self.threshold):
                                self.pixelRed = self.pixelRed + 1
                            if (frame_resized[j][i][2]<self.threshold):
                                self.pixelGreen = self.pixelGreen + 1

                    if (self.pixelRed>self.pixelGreen):
                        color = 'vermelho'
                        print("Pixels vermelhos",self.pixelRed)
                        print("Pixels verdes",self.pixelGreen)
                        self.camera_image.release()
                        return TriggerResponse(success=True, message=color)

                    else:
                        color = 'verde'
                        print("Pixels vermelhos",self.pixelRed)
                        print("Pixels verdes",self.pixelGreen)
                        self.camera_image.release()
                        return TriggerResponse(success=True, message=color)

        except KeyboardInterrupt:
            self.camera_image.release()

        self.camera_image.release()
        color="sem_cor"
        return TriggerResponse(success=True, message=color)

################       
# --- MAIN --- #
################

def main():
    rospy.init_node('camera_srvs')
    camera = Camera()
    rospy.loginfo('Start camera node')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        rate.sleep()

if (__name__ == '__main__'):
    main()