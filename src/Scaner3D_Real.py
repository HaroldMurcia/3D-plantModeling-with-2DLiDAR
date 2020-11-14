import rospy, os, sys, time, psutil
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import *
import math
from math import radians
import serial
import pandas as pd

from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

path_data = os.getcwd()
Arduino = serial.Serial('/dev/ttyUSB0', 9600)
pose = 0.0
theta =0.0
cont = 0.0
FLAG = 0
dZ = 0
dX = 0.75
dY = 0
dLim =dX+0.05

class scan():
    def __init__(self):

        self.delta_alfa = 726
        self.datos = np.zeros([self.delta_alfa,20])
        self.my_scan = np.zeros([726,1])
        self.counter=0
        self.Blue = np.zeros([1,726])
        self.Green = np.zeros([1,726])
        self.Red = np.zeros([1,726])
        self.bridge = CvBridge()
        rospy.loginfo("Starting node")
        rospy.init_node('scanner', anonymous=True)
        self.pub = rospy.Publisher('angle', String, queue_size=1)
        rospy.Subscriber('/scan',LaserScan, self.read_scan, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw',Image, self.captura_imagen, queue_size=1)
        self.syncrony_flag=0
        while not rospy.is_shutdown():
            self.mov_motor()

    def read_scan(self,data):
        global l,Xo,Yo,Zo, theta,FLAG, dX, dY, dZ,dLim
        if FLAG == 0:
            s = np.ones([len(data.ranges),1])
            s[:,0]=data.ranges
            self.my_scan=s[0:726,0]
            alfa = np.linspace(-120,120,self.delta_alfa )
            #print s[363]
            if self.syncrony_flag==0:
                # limita el rango de vision
                for i in range(len(self.my_scan)):
                    #alfa[i] = 120-0.3314917127*(i)
                    if np.isnan(self.my_scan[i])  or self.my_scan[i] > dLim  or self.my_scan[i] < 0.06:
                        self.my_scan[i] = np.nan
                self.datos[:,self.counter] = self.my_scan
                self.counter+=1

                #print self.my_scan[363]
                if self.counter==20:
                    self.counter=0
                    promedio=np.zeros([self.delta_alfa ,1])
                    for k in range(0,self.delta_alfa ):
                        promedio[k]+=np.nanmean(self.datos[k,:])
                    #print(promedio.T)
                    alfa = (alfa*math.pi)/180.0
                    ang_theta = (theta*math.pi)/180.0
                    # Matriz de datos
                    datos = np.zeros([4,self.delta_alfa ])
                    datos[0,:]=-promedio[:,0].T*np.cos(alfa)
                    datos[2,:]=promedio[:,0].T*np.sin(alfa)
                    datos[3,:]=np.ones([1,self.delta_alfa ])
                    # Matriz de translacion
                    M1 = np.array([[1,0,0,dX],[0,1,0,dY],[0,0,1,dZ],[0,0,0,1]])
                    # Matriz de rotacion
                    M2 = np.array([[np.cos(ang_theta),-np.sin(ang_theta),0,0],[np.sin(ang_theta),np.cos(ang_theta),0,0],[0,0,1,0],[0,0,0,1]])
                    out1 = np.dot(M1,datos)
                    out = np.dot(M2,out1)
                    Xo = out[0,:]
                    Yo = out[1,:]
                    Zo = out[2,:]
                    l = len(self.my_scan)
                    self.datos = np.zeros([self.delta_alfa ,20])
                    self.syncrony_flag=1

    def captura_imagen(self,data):
        cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        self.Blue = cv2_img[:,307,0]
        self.Green = cv2_img[:,307,1]
        self.Red = cv2_img[:,307,2]
        #print cv2_img[155,350,0]

    def mov_motor(self):
        global Xo, Yo, Zo, l, theta, cont,pose
        tic = time.time()
        arg = sys.argv
        delta_degree= float(arg[3])
        Max_degree  = float(arg[2])
        delta_steps = int(delta_degree*3600000) #pasar de grados a pasos del motor
        Max_steps   = int((Max_degree)*3600000)
        if (Max_steps > cont) and self.syncrony_flag==1:
            Arduino.write(str(0))
            pose = float(Arduino.readline())
            # Garantiza moviento de un delta_deg
            while abs(delta_degree-float(pose)) > 0.0:
                Arduino.write(str(delta_degree))
                pose = float(Arduino.readline())
            # Actualiza promedio de lecturas LiDAR
            Xa = Xo[260:480]
            Ya = Yo[260:480]
            Za = Zo[260:480]
            red = self.Red
            green = self.Green
            blue = self.Blue
            # Registro de datos en txt
            delta_image = 2

            for i in range (len(Xa)):
                Xx = Xa[i]
                Yy = Ya[i]
                Zz = Za[i]
                Red = red[i*delta_image]
                Green = green[i*delta_image]
                Blue = blue[i*delta_image]
                self.dataLine = str(Xx) + "\t"+ str(Yy) + "\t"+ str(Zz) + "\t"+ str(Red)+"\t"+ str(Green)+"\t"+ str(Blue) +"\n" # Escribe los datos X Y Z
                f.write(self.dataLine)

            cont += delta_steps
            theta+= delta_degree
            rospy.loginfo("Theta: "+str(theta))
            #time.sleep(0.2)
            self.syncrony_flag=0
            toc = time.time()
            tiempo = toc - tic
        if Max_steps  < cont:
            FLAG = 1

def saveCloud_txt(fileName):
    global completeName
    fileName = fileName
    completeName = os.path.join(path_data,fileName + '.txt') #crea el archivo .txt
    f = open(completeName,"a")
    return f

if __name__ == '__main__':
    fileName = str(sys.argv[1])
    f = saveCloud_txt(fileName)
    sc = scan()
    print "termino"
