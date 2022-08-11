#!/usr/bin/env python3
import rospy
import sys
import tf
import tf.msg
import qrcode
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
import geometry_msgs.msg
import numpy as np
import math
from random import seed
from random import random
import yaml
from pynput import keyboard     # for the keystroke to save the qr_tf coordinates for the 2dmap .yaml

t_previous = 0.0
ty_previous = 0.0
tz_previous = 0.0
rx_previous = 0.0
ry_previous = 0.0
rz_previous = 0.0
rw_previous = 0.0
qr_dict = {}
myData = ''


def on_press(key):
    print('{0} pressed'.format(key))
    if key == keyboard.KeyCode(char='y'):
        with open('/home/adricort/catkin_ws/src/rovy/2d_maps/qr_tf.yaml', 'w') as outfile:
            yaml.dump(qr_dict, outfile, default_flow_style=False)
        print("qr_tf coordinates yaml saved! :)")


def process_image(msg):
    global tx_previous,ty_previous,tz_previous
    global rx_previous,ry_previous,rz_previous, rw_previous
    global i, myData
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # QR CODE READER (with camera)
    if decode(img) == [] and myData != '':
        for keys, values in qr_dict.items():
            tx = values[0]
            ty = values[1]
            tz = values[2]
            rx = values[3]
            ry = values[4]
            rz = values[5]
            rw = values[6]

            '''br.sendTransform((tx, ty, tz),
                    (rx, ry, rz, rw),
                    rospy.Time.now(),
                    keys,
                    "map")'''

            t.header.frame_id = keys
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "map"
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x = rx
            t.transform.rotation.y = ry
            t.transform.rotation.z = rz
            t.transform.rotation.w = rw

            tfm = tf.msg.tfMessage([t])
            pub_qr.publish(tfm)

            #(trans,rot) = listener.lookupTransform('/map', keys, rospy.Time(0)) PENDIENTE!

    else:
        for barcode in decode(img):
            #print(barcode.data)    # shows the content (still coded)
            #print(barcode.rect)    # shows the position and size

            myData = barcode.data.decode('utf-8')  # decodes the data

            # Show polygon of QR code detection
            pts = np.array([barcode.polygon], np.int32)
            pts = pts.reshape(-1,1,2)
            cv2.polylines(img,[pts],True,(255,0,255),5)
            pts2 = barcode.rect
            cv2.putText(img,myData,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,(255,0,255),2)

            t.header.frame_id = myData
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "camera_link"
            t.transform.translation.x = (445-(barcode.rect.width+barcode.rect.height)/2)/400
            t.transform.translation.y = -(barcode.rect.left-255)/400
            t.transform.translation.z = -(barcode.rect.top-150)/400

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0 

            if qr_dict.get(myData) == None:
                qr_dict[myData] = [t.transform.rotation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z,
                                    t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w]

            #print(qr_dict)
            #print(myData)          # shows the content (decoded)

            # let's create the yaml file where qr_tf coordinates are stored for 2dmap

            tx_previous = t.transform.translation.x
            ty_previous = t.transform.translation.y
            tz_previous = t.transform.translation.z
            rx_previous = t.transform.rotation.x
            ry_previous = t.transform.rotation.y
            rz_previous = t.transform.rotation.z
            rw_previous = t.transform.rotation.w

            br.sendTransform((t.transform.translation.x, 
                            t.transform.translation.y,
                            t.transform.translation.z),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            myData,
                            "camera_link")
    
            tfm = tf.msg.tfMessage([t])
            pub_qr.publish(tfm)

    rate.sleep()

    #cv2.imshow("image",img)
    pub_image.publish(bridge.cv2_to_imgmsg(img))
    #print(img[0][0])
    cv2.waitKey(1)

if __name__ == '__main__':

        rospy.init_node('qr_tf_broadcaster_fromimage')
        rospy.loginfo('qr_tf_broadcaster_fromimage started')
        rospy.Subscriber("/camera/color/image_raw", Image, process_image)
        pub_image = rospy.Publisher('/camera/color/image_qr', Image,queue_size=10)
        pub_qr = rospy.Publisher("/tf_qr", tf.msg.tfMessage,queue_size=10)
        rate = rospy.Rate(30.0)
        br = tf.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        listener = tf.TransformListener()

        # Created methods for saving the .yaml file of the qr_tf coordinates 2dmap
        # Collect events until released
        with keyboard.Listener(
                on_press=on_press) as listener_keyboard:
            listener_keyboard.join()
        # ...or, in a non-blocking fashion:
        listener_keyboard = keyboard.Listener(
            on_press=on_press)
        listener_keyboard.start()

        rospy.spin()