#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()
nao_bateu = True
cv_image = None
media = [0,0]
centro = [0,0]
centro1=[0,0]
atraso = 1.5E9
media1=[0,0]
maior_contorno=None
maior_contorno_area1=0
maior_area=0
dist = 0
pista = True


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

def scaneou(dado):
    global nao_bateu
    global dist
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("---------------")
    print("Dist",dado.ranges[0])
    if dado.ranges[0] <= 0.45:
        nao_bateu=False
    


def identifica_creeper(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);
    cor_menor = np.array([0, 50, 50])
    cor_maior = np.array([4, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)



    centro_frame = (frame.shape[1]//2, frame.shape[0]//2)

    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length)
    

    maior_contorno = None;
    maior_contorno_area1=0;

    for cnt in contornos:
        area=cv2.contourArea(cnt)
        if area > maior_contorno_area1:
            maior_contorno=cnt
            maior_contorno_area1=area
    
    if not maior_contorno is None:
        cv2.drawContours(frame,[maior_contorno],-1,[0,0,255],5);
        maior_contorno=np.reshape(maior_contorno, (maior_contorno.shape[0],2));
        media1=maior_contorno.mean(axis=0);
        media1=media1.astype(np.int32)
        cv2.circle(frame,(media[0],media[1]), 5,[0,255,0])
        cross(frame, centro, [255,0,0],1,17)
    else:
        media1=(240,320)

    cv2.imshow('vermelho', segmentado_cor)
    
    return media1,maior_contorno_area1
    



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global cv_image
    global media
    global centro
    global media1
    global maior_area

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    #print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        #print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image2 = cv_image.copy();
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real

        media, centro =  cormodule.identifica_cor(cv_image) #puxa arquivo cormodule.py (centro = centro da image) (media= centro do maior contronro da cor desejada) (maior_area= tamanaho da area d maior contorno)
        
        media1,maior_area = identifica_creeper(cv_image2)


        depois = time.clock()

        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed" 

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    v=0.1
    tempo = 1.0/v
    tol = 32
    tol2 =15
    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            rospy.sleep(0.1)
            if len(media) != 0 and len(centro) != 0: #se existe uma media e um centro 
                if maior_area<=500 and pista: #se nao tiver identificado o creeper rosa
                    if (media[0] > centro[0]+tol):  #se a media do objeto identificado esta a direita do centro, gire a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.07))
                        print(media[0])
                    elif (media[0] < centro[0]+tol):  #se a media do objeto identificado esta a esquerda do centro, gire a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.07))
                        print(media[0])
                    if (media[0]-tol < centro[0]<media[0]+tol):
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                        #print(media[0segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))])
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.03)
                elif maior_area >500 and nao_bateu: #se tiver identificado o creeper
                    pista=False
                    print("IDENTIFICOU CREEPER")
                    print("IDENTIFICOU CREEPER")
                    print("IDENTIFICOU CREEPER")
                    print("IDENTIFICOU CREEPER")
                    print("IDENTIFICOU CREEPER")

                    if (media1[0] > centro[0]+tol2):  #se a media do objeto identificado esta a direita do centro, gire a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.04))
                        print(media[0])
                    elif (media1[0] < centro[0]+tol2):  #se a media do objeto identificado esta a esquerda do centro, gire a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.04))
                        print(media[0])
                    if (media1[0]-tol2 < centro[0]<media1[0]+tol2):
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                   
                        #print(media[0segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))])
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.02)

                elif nao_bateu==False and pista==False:
                    print('ATRASSASARSTARSTRASTRATSRA')
                    vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(tempo)
                    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(zero)
                    rospy.sleep(0.05)
                    nao_bateu=False
                    pista=True
                    if pista:
                        if (media[0] > centro[0]+tol):  #se a media do objeto identificado esta a direita do centro, gire a direita
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.07))
                            print(media[0])
                        elif (media[0] < centro[0]+tol):  #se a media do objeto identificado esta a esquerda do centro, gire a esquerda
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.07))
                            print(media[0])
                        if (media[0]-tol < centro[0]<media[0]+tol):
                            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                            #print(media[0segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))])
                        velocidade_saida.publish(vel)
                        rospy.sleep(0.03)

            else:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                velocidade_saida.publish(vel)
                rospy.sleep(0.03)

                velocidade_saida.publish(vel)
                rospy.sleep(0.03)
            
            

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


        