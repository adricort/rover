#!/usr/bin/env python3

import roslib
import rospy
import tf

import qrcode
import cv2
from pyzbar.pyzbar import decode
import numpy as np

cap = cv2.VideoCapture(4) # 4 es para RGB-D, 1 nube de puntos y 0 webcam
cap.set(3,640)
cap.set(4,480)

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        success, img = cap.read() 

        # QR CODE READER (with camera)
        for barcode in decode(img):
            #print(barcode.data)    # shows the content (still coded)
            print(barcode.rect)    # shows the position and size

            myData = barcode.data.decode('utf-8')  # decodes the data
            print(myData)          # shows the content (decoded)

            # Show polygon of QR code detection
            pts = np.array([barcode.polygon], np.int32)
            pts = pts.reshape(-1,1,2)
            cv2.polylines(img,[pts],True,(255,0,255),5)
            pts2 = barcode.rect
            cv2.putText(img,myData,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,(255,0,255),2)

            br.sendTransform(((445-(barcode.rect.width+barcode.rect.height)/2)/100, 
                            -(barcode.rect.left-255)/100, -(barcode.rect.top-255)/100),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "base_qr",
                            "base_link")
            rate.sleep()

        cv2.imshow('Result',img)
        print(img[0][0])

        cv2.waitKey(1)