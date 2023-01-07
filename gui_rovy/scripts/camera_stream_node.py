#!/usr/bin/python3
import cv2
import sys
import numpy as np
import math
import os
import time
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
import queue
import threading

color_data_array = [0.0,0.0,0.0]

# Camera config 0 to laptop, 2 to logitech
camera_device = 0
width = 320
height = 240
fps = 30
video_feed_publish_fps = 3 # 1 frame per second is sent on the ROS topic  
#amarillo = np.array([31,61,128,132,149,255])

# Perfil de color naranja
naranja = np.array([0, 255, 172, 255, 0, 255]) #Anaranjado para nublado
#negro = np.array([0, 38, 0, 255, 0, 255])
#naranja = negro # solo para no cambiar las variables
#verde_chingamelapupila = np.array([137, 255, 89, 127, 52, 103])
#naranja = np.array([47, 121, 168, 255, 0, 154])	#Anaranjado para sombra
# [lowYCon, highYCon, lowCrCon, higCrCon, lowCbCon, higCbCon] 

# Perfil de color de agua
#azul = np.array([100,255,99,125,171,255])
# [lowYWat, highYWat, lowCrWat, higCrWat, lowCbWat, higCbWat]
 	
# Limites del area aceptable para reconocimiento
# El area maxima de la imagen es de width*(height) pixeles
# el arreglo esta ordenado como [minArea, maxArea]
areaNar = np.array([70, 45000])

# Variables para hacer la malla de localizacion
# Se esta respetando el origen superior izquierdo en la imagen
# limites del intervalo de las secciones horizontales
xSec = np.array([45, 90, 135, 185, 230, 275])
# limites del intervalo de las secciones verticales
ySec = np.array([200, 160, 120, 80, 40])

# Vector de maximos valores acumulados para cada uno de
# los colores de interes, durante un muestreo de 20 frames
# El objetivo, es clasificar el color detectado, acorde al
# comportamiento estadistico de las observaciones registradas
colorClases = np.array([20, 40, 60])
#(max.de azul, max. de rojo, max. de negro)

# Seleccion del orden de los pines
#gpio.setmode(gpio.BOARD)
# desacticacion de las notificaciones warning
#gpio.setwarnings(False)

# Pines GPIO a utilizar
ledR = 13
ledG = 11
ledB = 15
pinOnOFF = 18
pinStart = 12

# Configuracion de los pines
#gpio.setup(ledR, gpio.OUT)
#gpio.setup(ledG, gpio.OUT)
#gpio.setup(ledB, gpio.OUT)

#gpio.setup(pinOnOFF,gpio.IN, pull_up_down = gpio.PUD_UP)
#gpio.setup(pinStart,gpio.IN)

# Inicializacion de los pines GPIO
#gpio.output(ledR, False)
#gpio.output(ledG, False)
#gpio.output(ledB, False)

saturacion = 0

def CompressionService(cvImgQueue, pub_video):

    print("Initializing image compression service")

    while True:
        # block until new image to compress
        rawImg = cvImgQueue.get()
        now = rospy.Time.now()
        print("Compressing image at timestamp " + str(time.time()))

        # Create CompressedImage
        msg = CompressedImage()
        msg.header.stamp = now
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', rawImg)[1]).tobytes()
        
        # publish
        pub_video.publish(msg)


def prevFilter(img):
    # Ventana para aplicar la el filtro de dilatcion y erosion
    wKernel = np.ones((5,5),np.uint8)
    # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
    fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
    # Filtro mediana de suavizamiento de imagen
    fImg = cv2.medianBlur(fImg, 5)
    #print('filtrado')
    #cv2.imshow('alrs',fImg)
    return fImg



def getMask(img, pfColor):
	
    # Define el conjunto de valores para obtener la mascara binaria
    lowLimits = [pfColor[0],pfColor[2],pfColor[4]]
    highLimits = [pfColor[1],pfColor[3],pfColor[5]]

    # Calculo de la mascara binaria, en funcion de los limites definidos
    mask = cv2.inRange(img, np.float32(lowLimits), np.float32(highLimits))

    # Ajuste de mascara
    # Se define la ventana a utilizar en el filtro morfologico
    wKernel = np.ones((9,9),np.uint8)
    # Filtro de dilatacion y erosion a la mascara
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, wKernel)
    #cv2.imshow('mask', mask)

    return mask



def getCentroids(img, areaRef):
    # Inicializa lista para guardar los centros que se calcularan
    centers = []

    # Obtencion de segmentos, basado en la imagen binaria
    contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    # Ordenamiento de mayor a menor de todos los segmentos encontrados
    # considerando como criterio su area. Solo se conserva el mayor
    # el cual se controla, por el [:1] al final de la instruccion
    contours = sorted(contours, key=cv2.contourArea, reverse = True)[:1]
    #cv2.imshow('test2',img)

    for c in contours:
        # Calculo del area de un segmento
        blobArea = cv2.contourArea(c)
        global saturacion
        saturacion = blobArea
        #perAreaFind = int( blobArea/(areaRef[1] - areaRef[0]) )*100

        # Se busca que el area del segmento en estudio, este dentro del intervalo
        # establecido
        if ( (blobArea > areaRef[0]) and (blobArea < areaRef[1]) ): 
            # Calculo del momento del segmento encontrado
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"]/ M["m00"])
                cY = int(M["m01"]/ M["m00"])
                # Bandera para decir que el dato es util
                use = 1
                centers.append(cX)
                centers.append(cY)
                #centers.append((use,cX,cY, perAreaFind))
        else:
            cX = 0
            cY = 0
            # Bandera para decir que el dato es util
            use = 0

            centers.append(cX)
            centers.append(cY)
            #centers.append((use,cX,cY, perAreaFind))


    if not centers:
        centers.append(0)
        centers.append(0)

    return centers



def mapGrid(centers):
    # Obtiene el indice en donde debe de colocarse la abscisa
    # Al restar -3, da como resultado el sector en donde se
    # ubica la coordenada x del centroide (-3, -2, -1, 0, 1, 2, 3)
    xSector = np.searchsorted(xSec, centers[0][1])
    cv2.waitKey(2)
    # Obtiene el indice en donde debe de colocarse la ordenada
    # Al sumar 1, da como resultado el sector en donde se
    # ubica la coordenada y del centroide (1, 2, 3, 4, 5, 6)
    # La parte baja de la imagen es el sector 1, y asciende en
    # intervalos de 40 pixeles
    ySector = np.searchsorted(ySec, centers[0][2]) + 1
    cv2.waitKey(2)

    # Incializa lista para guardar los nuevos centros
    qCenters = []
    # Adjunta ambas coordenadas en un elemento de la lista
    qCenters.append((xSector, ySector))

    return qCenters


def findObjs(imgSrc):

    colorCorden = []
    # Aplica un filtro morfologico, para suavizar bordes
    fImg = prevFilter(imgSrc)
    #cv2.imshow('filter', fImg)

    # Conversion del espacio de colores a YCrCb
    yuvImg = cv2.cvtColor(fImg, cv2.COLOR_BGR2YCR_CB)

    ### Busqueda de la pelota - COLOR NARANJA
    # Obtiene la mascara para el color naranja
    naranMask = getMask(yuvImg, naranja)
    # Obtiene los centroides, de la segmentacion de la mascara
    cordNaran = getCentroids(naranMask, areaNar)
    #cv2.imshow('maskNaranja', naranMask) ################### THIS IS THE ONE THAT SHOWS THE WINDOW

    # Guarda datos dentro de la lista para enviarlos por serial
    colorCorden.append(1)
    colorCorden.append(cordNaran[0])
    colorCorden.append(cordNaran[1])


    ### Busqueda de la porteria azul - COLOR AZUL
    # Calcula la mascara para el color azul
    #azulMask = getMask(yuvImg, azul)
    # Obtiene los centroides, de la segmentacion de la mascara
    #cordAzul = getCentroids(azulMask, areaAzul)
    #cv2.imshow('maskAzul', azulMask)


    # Guarda datos dentro de la lista para enviarlos por serial
    #colorCorden.append(2)
    #colorCorden.append(cordAzul[0])
    #colorCorden.append(cordAzul[1])


    ### Busqueda de la porteria amarilla - COLOR AMARILLO
    # Obtencion la mascara para el color amarillo
    #amarMask = getMask(yuvImg, amarillo)
    # Obtiene los centroides, de la segmentacion de la mascara
    #cordAmar = getCentroids(amarMask, areaAmar)
    #cv2.imshow('maskAmarilla', amarMask)

    # Guarda datos dentro de la lista para enviarlos por serial
    #colorCorden.append(3)
    #colorCorden.append(cordAmar[0])
    #colorCorden.append(cordAmar[1])

    return colorCorden


def assignData(colorVector):
    color_data_array[0] = colorVector[1];
    color_data_array[1] = colorVector[2];
    color_data_array[2] = saturacion;

    if color_data_array[0] == 0 and color_data_array[1] == 0:
        color_data_array[2] = 0;

    #print("x-Axis Orange: ",color_data_array[0]," y-Axis Orange: ",color_data_array[1], " Saturation: ", color_data_array[2])

def dataSel(color, xsec, ysec):
    colorMean = np.mean(color)
    delta = abs(math.trunc(colorMean)-colorMean)

    if (delta <= 0.51):
        rColor = math.floor(colorMean)
    else:
        rColor = round(colorMean)

    xSecMean = np.mean(xsec)
    deltaX = abs(math.trunc(xSecMean)-xSecMean)

    if (deltaX <= 0.51):
        rSecX = math.floor(xSecMean)
    else:
        rSecX = round(xSecMean)

    ySecMean = np.mean(ysec)
    deltaY = abs(math.trunc(ySecMean)-ySecMean)

    if (deltaY <= 0.51):
        rSecY = math.floor(ySecMean)
    else:
        rSecY = round(ySecMean)

    return rColor, rSecX, rSecY

def main():

    # Name of the node. False = no random numbers
    rospy.init_node('color_detection_node', anonymous=False)
    print("Initializing color_detection_node...")

    # Published definition. Int32MultiArray because of PWM (0-255 int)
    pub_color = rospy.Publisher('/color_marker_topic', Float32MultiArray, queue_size=10)

    # Node is publishing to the video_frames topic using 
    # the message type Image
    pub_video = rospy.Publisher('/video_frames', CompressedImage, queue_size=1)

    # Data definition
    color_marker_data = Float32MultiArray()

    # Program frequency
    rate = rospy.Rate(70) # 50hz

    # Inicializacion de la camara
    vSrc = cv2.VideoCapture(camera_device)
    # Configuracion del tamano de los frames
    vSrc.set(cv2.CAP_PROP_FRAME_WIDTH,width)
    vSrc.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
    # Configuracion de la Tasa de muestreo de la camara
    vSrc.set(cv2.CAP_PROP_FPS,fps)
    #start = False
    #while ( start == False ):
    #	start = gpio.input(pinStart)
    #	cv2.waitKey(50)
    #	gpio.output(ledR, True)
    #	gpio.output(ledG, True)
    #	gpio.output(ledB, True)
    #	cv2.waitKey(100)
    #	gpio.output(ledR, False)
    #	gpio.output(ledG, False)
    #	gpio.output(ledB, False)
    #	cv2.waitKey(100)

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # start compression thread
    cvImgQueue = queue.Queue()
    compSvcThr = threading.Thread(target=CompressionService, args=(cvImgQueue,pub_video), daemon=True)
    compSvcThr.start()
    last_time = time.time()

    # ======================== While loop ===================================
    while not rospy.is_shutdown(): 
        try:
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            ret,imgSrc = vSrc.read()

            if ( ret ):
                # Print debugging information to the terminal

                cordSend = findObjs(imgSrc)

                assignData(cordSend)

                color_marker_data.data = color_data_array
                pub_color.publish(color_marker_data)

                # Publish the image if need be
                now = time.time()
                if now - last_time > (1.0 / video_feed_publish_fps):
                    cvImgQueue.put(imgSrc)
                    last_time = now

                rate.sleep()

                #gpio.output(ledG, True)
                #cv2.waitKey(100)
                #gpio.output(ledG, False)

                #cv2.imshow('original',imgSrc)
        except KeyboardInterrupt:
            vSrc.release()
            cv2.destroyAllWindows()
            #gpio.cleanup()
            sys.exit()

# ================= LOOP =========================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
