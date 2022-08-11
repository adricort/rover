#!/usr/bin/env python3

import rospy
import actionlib
import geometry_msgs.msg
import tf
import tf.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from actionlib_msgs.msg import GoalStatusArray

from cv_bridge import CvBridge, CvBridgeError
import os
from math import sqrt

import logging
import threading
import time
import array
import sys
import serial
import serial.tools.list_ports
import glob
import struct

av = [0,0]
lrmArray = [0,0,0,0]
angle = 0

#calibrated_angle_fb = [150,150,205,160,160,150]                 # Forward/Backwards calibrated angles
calibrated_angle_fb = [0,0,0,0,0,0]
calibrated_angle_rot_clk = [0,0,0,0,0,0]          # Calibrated rover rotation driving angles clockwise
calibrated_angle_rot_anticlk = [0,0,0,0,0,0]          # Calibrated rover rotation driving angles anticlockwise
calibrated_krabwalk = [0,0,0,0,0,0]          # Krabwalk dynamic angles

calibrated_angle_rot_anticlk[0] =  calibrated_angle_fb[0]-120
calibrated_angle_rot_anticlk[1] =  calibrated_angle_fb[1]-60
calibrated_angle_rot_anticlk[2] =  calibrated_angle_fb[2]+120
calibrated_angle_rot_anticlk[3] =  calibrated_angle_fb[3]+180
calibrated_angle_rot_anticlk[4] =  calibrated_angle_fb[4]
calibrated_angle_rot_anticlk[5] =  calibrated_angle_fb[5]+60

for i in range(6):
    if calibrated_angle_rot_clk[i] < 0:
        calibrated_angle_rot_clk[i] = calibrated_angle_rot_clk[i]+360
    elif calibrated_angle_rot_clk[i] > 360:
        calibrated_angle_rot_clk[i] = calibrated_angle_rot_clk[i]-360

calibrated_angle_rot_clk[0] =  calibrated_angle_rot_anticlk[0]+180
calibrated_angle_rot_clk[1] =  calibrated_angle_rot_anticlk[1]+180
calibrated_angle_rot_clk[2] =  calibrated_angle_rot_anticlk[2]+180
calibrated_angle_rot_clk[3] =  calibrated_angle_rot_anticlk[3]+180
calibrated_angle_rot_clk[4] =  calibrated_angle_rot_anticlk[4]+180
calibrated_angle_rot_clk[5] =  calibrated_angle_rot_anticlk[5]+180

for i in range(6):
    if calibrated_angle_rot_anticlk[i] < 0:
        calibrated_angle_rot_anticlk[i] = calibrated_angle_rot_anticlk[i]+360
    elif calibrated_angle_rot_anticlk[i] > 360:
        calibrated_angle_rot_anticlk[i] = calibrated_angle_rot_anticlk[i]-360

calibrated_pantilt = [130,180]
communication_time = 1./100.                   # GIVE TIME TO THE CONTROLLER TO REACH THE POINT! (1./100.)
speed_cruising = 10000
cmd = [0,0]
robot_pose = [0,0]
goal_pose = [0,0]
goal_status = 0

class LRM_DataStruct:
    def __init__(self):
        self._lock = threading.Lock()

        self.errorextern = 0    #errorvalue
        self.seterror = 0       #errorclearvalue for bogies

        self.bogie00 = self.Bogie(0,0)      #create all the bogies
        self.bogie01 = self.Bogie(0,1)
        self.bogie10 = self.Bogie(1,0)
        self.bogie11 = self.Bogie(1,1)
        self.bogie20 = self.Bogie(2,0)
        self.bogie21 = self.Bogie(2,1)
        self.bogies =[]                     #put the bogies in a list
        self.bogies.append(self.bogie00)    
        self.bogies.append(self.bogie01)
        self.bogies.append(self.bogie10)
        self.bogies.append(self.bogie11)
        self.bogies.append(self.bogie20)
        self.bogies.append(self.bogie21)

        self.pantilt = self.PanTilt()

    class Bogie:
        counter = 0    #instance counter
        def __init__(self, Bogie_ID, Wheel_ID):
            #set bogie data
            self.mode = 200     #default mode = normalmode
            self.bogie_ID = Bogie_ID
            self.wheel_ID = Wheel_ID
            self.setangle = 0
            self.setspeed = 0
            #get bogie data
            self.realangle = 0
            self.realspeed = 0
            self.anglebogie = 0         #only in bogiewheel X0 (X=0/1/2) ->see physical connection
            self.motorcurrentsteer = 0
            self.motorcurrentdrive = 0
            self.magencodersteerinc = 0 #magnetic encoder steer increments
            self.flagcounterdrive = 0
            self.errorintern = 0        #gets send as ERRtemp
            self.crc_check = 0
            #status bogie
            self.sendData = True
            self.setControllerOnOff = 1
            #PID controller data
            self.PID_data = self.PID_Data()

            type(self).counter += 1
            def __del__(self):
                type(self).counter -= 1

        class PID_Data:
            def __init__(self):
                #TODO: Note that these default values are also hard coded in the bogie code and where determined purely experimentally
                self.DP_i = 36  #Drive Parameter P inner loop       Drive PIDinit: 36, 4, 0,11,0,0, 65000 ,65000, 4, 2
                self.DI_i = 4  #Drive Parameter I inner loop
                self.DD_i = 0  #Drive Parameter D inner loop
                self.DP_o = 11  #Drive Parameter P outer loop
                self.DI_o = 0  #Drive Parameter I outer loop
                self.DD_o = 0  #Drive Parameter D outer loop
                self.SP_i = 4113  #Steer Parameter P inner loop     SteerPIDinit: 4113,0,0 ,8912,0,0, 65000 , 65000, 4, 1
                self.SI_i = 0  #Steer Parameter I inner loop
                self.SD_i = 0  #Steer Parameter D inner loop
                self.SP_o = 8912  #Steer Parameter P outer loop
                self.SI_o = 0  #Steer Parameter I outer loop
                self.SD_o = 0  #Steer Parameter D outer loop
                self.ScaleD_i = 4  #Drive Scaling inner loop
                self.ScaleS_i = 4  #Steer Scaling inner loop
                self.ScaleD_o = 2  #Drive Scaling outer loop
                self.ScaleS_o = 1  #Steer Scaling outer loop
                self.WD_o = 65000  #Drive Windup outer loop
                self.WD_i = 65000  #Drive Windup inner loop
                self.WS_o = 65000  #Steer Windup outer loop
                self.WS_i = 65000  #Steer Windup inner loop
                #list PID parameter for systematic access: S= Steer, D = Drive, I/i = Inner Loop, O/o = Outer Loop #TODO:simpify parameter names
                self.list_S_I = [self.SP_i, self.SI_i, self.SD_i, self.ScaleS_i, self.WS_i]
                self.list_S_O = [self.SP_o, self.SI_o, self.SD_o, self.ScaleS_o, self.WS_o]
                self.list_D_I = [self.DP_i, self.DI_i, self.DD_i, self.ScaleD_i, self.WD_i]
                self.list_D_O = [self.DP_o, self.DI_o, self.DD_o, self.ScaleD_o, self.WD_o]
                self.list_S_IO = [self.list_S_I, self.list_S_O]
                self.list_D_IO = [self.list_D_I, self.list_D_O]
                self.list_SteerDrive = [self.list_S_IO, self.list_D_IO]

    class PanTilt():
        def __init__(self):
            self.PanTiltset = 1
            self.Pan = 127  #init value 127 (0-255)
            self.Tilt = 127  #init value 127 (0-255)
# =============================================================================
#                               Communication
# =============================================================================
class Communication:
    def __init__(self):
        self.ser = 0
        self.selected_port = 0

    def connect(self):
        #get all the available ports and print them
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        for lauf in  result   :
            print(lauf)


        self.selected_port = 0 #input("Select Port: ")
        try:
            self.ser = serial.Serial(result[int(self.selected_port)])
            if self.ser.isOpen():
                print(self.ser.portstr + " is enabled!\n")

                self.ser.baudrate    = 230400
                self.ser.timeout     = 2
                self.ser.bytesize    = serial.EIGHTBITS
                self.ser.parity      = serial.PARITY_NONE
                self.ser.stopbits    = serial.STOPBITS_ONE
        except:
            print("could not connect to"+result[self.selected_port])

    def disconnect(self):
        try:
            self.ser.close()
            print("disconnected" + "(COM" + str(self.selected_port) + ")")
        except:
            print("Error with COM-Port connection!")

    def createmessage(self, datastruct):
        datastruct._lock.acquire()
        #bogie modes
        NormalMode = 200
        ErrorSetMode = 90
        CalibrationMode = 150
        ControllerOnOff = 120
        ControllerSet = 80

        message = bytearray(0)
        message.append(146) #select maintobogie protocol
        message.append(0)   #preserve the first two bytes for the message length
        message.append(0)

        #select mode dependent message for bogies
        for bogie in range(len(datastruct.bogies)):   #for every bogiewheel (6)
            if datastruct.bogies[bogie].sendData == True:
                if datastruct.bogies[bogie].mode == NormalMode:
                    sendNormalMode(message,
                                   datastruct.bogies[bogie].wheel_ID,
                                   datastruct.bogies[bogie].setangle,
                                   datastruct.bogies[bogie].setspeed,
                                   datastruct.errorextern)
                elif datastruct.bogies[bogie].mode == ErrorSetMode:
                    sendErrorSetMode(message,
                                     datastruct.bogies[bogie].wheel_ID,
                                     datastruct.seterror,
                                     datastruct.errorextern)
                elif datastruct.bogies[bogie].mode == CalibrationMode:
                    sendCalibrationMode(message,
                                        datastruct.bogies[bogie].wheel_ID,
                                        datastruct.errorextern)
                elif datastruct.bogies[bogie].mode == ControllerOnOff:
                    sendControllerOnOff(message,
                                        datastruct.bogies[bogie].wheel_ID,
                                        datastruct.bogies[bogie].setControllerOnOff,
                                        datastruct.errorextern)
                elif datastruct.bogies[bogie].mode == ControllerSet:
                    sendControllerSet(message,
                                      datastruct.bogies[bogie].wheel_ID,
                                      datastruct.bogies[bogie].PID_data,
                                      datastruct.errorextern)
                else:
                    print("Error: no/wrong mode selected")
                    raise ValueError
            else:
                message.append(0)

        if datastruct.pantilt.PanTiltset == 1:
            #print(datastruct.pantilt.Pan)
            #print(datastruct.pantilt.Tilt)
            message.append(1)
            message.append(130)
            message.append(6)
            message.append(datastruct.pantilt.Pan)  #pan
            message.append(datastruct.pantilt.Tilt)  #tilt
            message.append(0)  #Error
            message.append(0)  #CRC
        else:
            message.append(1)

        #last bits no data (for now)
        message.append(0) #TODO: IMU
        message.append(0) #Middle Board
        message.append(0) #Error
        message.append(0) # TODO: CRC

        #determine message length
        message[1] = ((len(message) & 255))
        message[2] = ((len(message) >>8))

        datastruct._lock.release()
        return message #return finished message

    def SendData(self, data): #send LRM data
        try:
            self.ser.flushInput()
            for i in data:
                self.ser.write(struct.pack('>B', i))
                #print(i, end=" ")
            print("")
        except:
            print("Error: Attempting to use a port that is not open")

    def ReadData(self): #returns body message
        print("")               

# =============================================================================
#                               ModeHandleing
# =============================================================================
def BogieModusSort(DataBogie,bogie,datastruct):
    datastruct._lock.acquire()
    NormalMode = 200
    ErrorSetMode = 90
    CalibrationMode = 150
    ControllerOnOff = 120
    ControllerSet = 80
    i = 0
    modus = DataBogie[i] & 0xfe
    i += 2 #skipp mode + length
    if NormalMode == modus:
        datastruct.bogies[bogie].realangle = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].realspeed = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].anglebogie = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].motorcurrentsteer = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].motorcurrentdrive = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].magencodersteerinc = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].flagcounterdrive = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].errorintern = DataBogie[i]
        i += 1
        datastruct.bogies[bogie].crc_check = DataBogie[i]
    elif ErrorSetMode == modus:
        datastruct.bogies[bogie].errorintern = DataBogie[i]
        i += 1
        #datastruct.errorextern = DataBogie[i]   #TODO: should be errorextern
        i += 1
        datastruct.bogies[bogie].crc_check = DataBogie[i]
    elif CalibrationMode == modus:
        datastruct.bogies[bogie].errorintern = DataBogie[i]
        i += 1
        datastruct.bogies[bogie].crc_check = DataBogie[i]
    elif ControllerOnOff == modus:  
        #ControllerOnOff same Data as with NormalMode
        datastruct.bogies[bogie].realangle = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].realspeed = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].anglebogie = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].motorcurrentsteer = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].motorcurrentdrive = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].magencodersteerinc = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].flagcounterdrive = DataBogie[i] | (DataBogie[i+1] << 8)
        i += 2
        datastruct.bogies[bogie].errorintern = DataBogie[i]
        i += 1
        datastruct.bogies[bogie].crc_check = DataBogie[i]
    elif ControllerSet == modus:
        datastruct.bogies[bogie].errorintern = DataBogie[i]
        i += 1
        datastruct.bogies[bogie].crc_check = DataBogie[i]
    else:
        logging.info("Error unknown mode received: " + str(modus))
    datastruct._lock.release()
# =============================================================================
# -------------------------------SendModes
# =============================================================================
def sendNormalMode(msg,wheel,angleint,speedint,ErrorExtern):
    msg.append(1) #sendData == True
    msg.append(200+wheel)
    msg.append(8)
    msg.append(angleint & 255) #angle low byte
    msg.append(angleint >> 8) #angle high byte
    msg.append(speedint & 255) #speed low byte
    msg.append(speedint >> 8) #speed high byte
    msg.append(ErrorExtern)
    msg.append(0)

def sendErrorSetMode(msg,wheel,SetError,ErrorExtern):
    msg.append(1) #sendData == True
    msg.append(90+wheel)
    msg.append(6)
    msg.append(SetError)
    msg.append(0)
    msg.append(ErrorExtern)
    msg.append(0)

def sendCalibrationMode(msg,wheel,ErrorExtern):
    msg.append(1) #sendData == True
    msg.append(150+wheel)
    msg.append(4)
    msg.append(ErrorExtern)
    msg.append(0)

def sendControllerOnOff(msg,wheel,Controller_OnOff,ErrorExtern):
    msg.append(1) #sendData == True
    msg.append(120+wheel)
    msg.append(5)
    msg.append(Controller_OnOff) #1=controller on, 0=controller off
    msg.append(ErrorExtern)
    msg.append(0)

# =============================================================================
# ----------------------------user-functions
# =============================================================================

def setangle(bogiewheel,angle,datastruct,com):
    datastruct.bogies[bogiewheel].mode=90
    #for i in range(2):
    com.SendData(com.createmessage(datastruct))
    #showbogiedata(datastruct)
    time.sleep(1./1000.)
    #com.ReceiveData(com.ReadData(),datastruct)

    datastruct.bogies[bogiewheel].mode=200
    datastruct.bogies[bogiewheel].setangle = angle
    #for i in range(31):
    com.SendData(com.createmessage(datastruct))
    #showbogiedata(datastruct)
    time.sleep(1./1000.)
    #time.sleep(1./1000.)
    #com.ReceiveData(com.ReadData(),datastruct)
    #showbogiedata(datastruct)

def setdriving(drivingmode,angle,speed,datastruct,com,delay):
    if drivingmode == 0:                                    # Ackerman mode
        if speed < 0:
            speed=int(bin(2**16+speed)[-16:],2)
        for i in range (delay):
            for i in range (6):
                datastruct.bogies[0].setangle = 360-angle
                datastruct.bogies[1].setangle = 360-angle
                datastruct.bogies[2].setangle = angle
                datastruct.bogies[3].setangle = 0
                datastruct.bogies[4].setangle = 0
                datastruct.bogies[5].setangle = angle
                datastruct.bogies[i].setspeed = speed
            com.SendData(com.createmessage(datastruct))
            #showbogiedata(datastruct)
            time.sleep(communication_time)
    elif drivingmode == 1:                                  # rotation mode clockwise
        speed_negative=int(bin(2**16-speed)[-16:],2)
        for i in range (delay):
            for i in range (6):
                datastruct.bogies[i].setangle = calibrated_angle_rot_clk[i]
                datastruct.bogies[i].setspeed = speed
            com.SendData(com.createmessage(datastruct))
            #showbogiedata(datastruct)
            time.sleep(communication_time)
    elif drivingmode == 2:                                  # rotation mode anticlockwise
        #speed_negative=int(bin(2**16-speed)[-16:],2)
        for i in range (delay):
            for i in range (6):
                datastruct.bogies[i].setangle = calibrated_angle_rot_anticlk[i]
                datastruct.bogies[i].setspeed = speed
            com.SendData(com.createmessage(datastruct))
            #showbogiedata(datastruct)
            time.sleep(communication_time)
    elif drivingmode == 3:                                    # krabwalk mode
        if speed < 0:
            speed=int(bin(2**16+speed)[-16:],2)
        for i in range (delay):
            for i in range (6):
                calibrated_krabwalk[i] = angle+calibrated_angle_fb[i]
                if calibrated_krabwalk[i] < 0:
                    calibrated_krabwalk[i] = calibrated_krablwalk[i]+360
                elif calibrated_krabwalk[i] > 360:
                    calibrated_krabwalk[i] = calibrated_krabwalk[i]-360
                datastruct.bogies[i].setangle = calibrated_krabwalk[i]
                datastruct.bogies[i].setspeed = speed
                calibrated_krabwalk[i] = 0
            com.SendData(com.createmessage(datastruct))
            #showbogiedata(datastruct)
            time.sleep(communication_time)

def setstop(datastruct,com,delay):
    for j in range (delay):
        for i in range (6):
            datastruct.bogies[i].setspeed = 0
        com.SendData(com.createmessage(datastruct))
        #showbogiedata(datastruct)
        time.sleep(communication_time)

def setspeed(bogiewheel,speed,datastruct,com):
    datastruct.bogies[bogiewheel].mode=90
    #for i in range(2):
    com.SendData(com.createmessage(datastruct))
    #showbogiedata(datastruct)
    time.sleep(1./1000.)
    #com.ReceiveData(com.ReadData(),datastruct)

    datastruct.bogies[bogiewheel].mode=200
    datastruct.bogies[bogiewheel].setspeed = speed
    #for i in range(31):
    com.SendData(com.createmessage(datastruct))
    #showbogiedata(datastruct)
    time.sleep(1./1000.)
    #com.ReceiveData(com.ReadData(),datastruct)

def showbogiedata(datastruct):
    #print("bogienum:", datastruct.bogies[i].bogie_ID, end=" ")
    #print("wheelnum:", datastruct.bogies[i].wheel_ID, end=" ")
    #print("setangle:", datastruct.bogies[i].setangle, end=" ")
    #print("setspeed:", datastruct.bogies[i].setspeed, end=" ")
    #print("pan:", datastruct.pantilt.Pan, end=" ")
    #print("tilt:", datastruct.pantilt.Tilt, end=" ")

    #print("v2 ",speed," a2: ",angle,"|v0 ",speed_wheels[1]," a0: ",angle_wheels[0])
    #print("v3 ",speed," a3: ",angle_wheels[0],"|v1 ",speed_wheels[1]," a1: ",angle_wheels[0])
    #print("v5 ",speed," a5: ",angle_wheels[0],"|v4 ",speed_wheels[1]," a4: ",angle_wheels[0])

    #print("realangle:", datastruct.bogies[i].realangle, end=" ")
    #print("realspeed:", datastruct.bogies[i].realspeed, end=" ")
    #print("anglebogie:", datastruct.bogies[i].anglebogie, end=" ")
    #print("currentsteer:", datastruct.bogies[i].motorcurrentsteer, end=" ")
    #print("currentdrive:", datastruct.bogies[i].motorcurrentdrive, end=" ")
    #print("magencsteerinc:", datastruct.bogies[i].magencodersteerinc, end=" ")
    #print("flagcounterdrive:", datastruct.bogies[i].flagcounterdrive, end=" ")
    #print("errorintern:", datastruct.bogies[i].errorintern, end=" ")
    #print("crc_check:", datastruct.bogies[i].crc_check)
    print("showingbogiedata")
# =============================================================================
#                                   Threads
# =============================================================================
def stopcom():
    global ComOnOff
    ComOnOff = False

def startcom():
    global ComOnOff
    ComOnOff = True
    ISR_com = threading.Thread(target=ISR_COM, args=(1,))
    ISR_com.start()


def move_base_lrm(cmd_data):
    cmd[0] = cmd_data.linear.x        # This is the linear velocity that Twist provides for the planning (max 0.25)
    cmd[1] = cmd_data.angular.z       # This is the angular velocity that Twist provides for the planning (range not known, < 1.0 maybe)

    # 0.25* = 10000 (max could reach 15000, but let's do it slow)
    cmd[0] = cmd[0]*40000     # velocity
    cmd[1] = cmd[1]*100       # angle (ideally it is angular velocity, but it is used as angle, just to test)

def odomCallback(odom_msg):
    robot_pose = odom_msg.transforms.transform.translation.x
    #robot_pose[1] = odom_msg.transforms .transform.translation.y
    print(robot_pose)
    #distance = sqrt(pow(goal_pose[0] - robot_pose[0], 2) + pow(goal_pose[1] - robot_pose[1], 2))
    #print(distance)
    #if distance < 0.5:
    #    move_base.cancel_goal()
    #    rospy.loginfo("goal has been canceled")

def goalCallback(goal_msg):
    goal_pose[0] = goal_msg.pose.position.x
    goal_pose[1] = goal_msg.pose.position.x
    #print(goal_pose)

def goalStatusCallback(goal_status_msg):
    goal_status = goal_status_msg.status_list[0].status

# ======================== While loop ===================================
def main():
    rospy.init_node('navigation_node', anonymous=False)
    move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.Subscriber('/cmd_vel',Twist,move_base_lrm)
    rospy.Subscriber('/move_base/status',GoalStatusArray,goalStatusCallback)
    rospy.Subscriber('/move_base/goal',PoseStamped,goalCallback)    # status of the navigation 1 (ACTIVE), 3 (GOAL REACHED), given by ROS
                                                                    #  http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html


    lrm = LRM_DataStruct()
    lrm_rosdata = Int32MultiArray()

    c=Communication()
    c.connect()

    # Set to error, like a reset, to start the normal mode afterwards
    for i in range(6):
        lrm.bogies[i].mode=90
    c.SendData(c.createmessage(lrm))
    #showbogiedata(lrm)
    time.sleep(communication_time)

    # Set normal mode
    for j in range(30):
        for i in range(6):
            lrm.bogies[i].mode=200
        lrm.pantilt.Pan = calibrated_pantilt[0]
        lrm.pantilt.Tilt = calibrated_pantilt[1]
        c.SendData(c.createmessage(lrm))
        #showbogiedata(lrm)
        time.sleep(communication_time)

    rate = rospy.Rate(15)

    for k in range (30):
        setstop(lrm,c,1)

    start = time.time()
    velocity = 0
    angle = 0
    recovery_status = 0
    rotation_status = 0
    left_flag = 0
    right_flag = 0

    while not rospy.is_shutdown():
        # DEMO
        '''setdriving(0,0,10000,lrm,c,50)
        setdriving(1,0,10000,lrm,c,50)
        setdriving(2,0,10000,lrm,c,50)
        setdriving(3,150,10000,lrm,c,50)'''

        #lrm_rosdata = lrm

        '''if cmd[1] > -30 and cmd[1] < 30:        # +/- 40 was the tolerance value found with the configured proportional for angle form Twist
            setdriving(0,0,int(cmd[0]),lrm,c,1)
            #print("FORWARDS/BACKWARDS")
        elif cmd[1] <= -30:
            setdriving(1,0,4000,lrm,c,20)
            #print("TURNING LEFT")
        elif cmd[1] >= 30:
            setdriving(2,0,4000,lrm,c,20)
            #print("TURNING RIGHT")'''

        end = time.time()
        time_passed = end-start
        # Maximum value for angle from the New Plan is +/- 31.999
        # When there is recovery status, it only gives angle = 100
        if int(cmd[1]) == 100:
            recovery_status = 1
        else:
            recovery_status = 0

        if time_passed > 1.0:
            velocity = int(float(cmd[0])*2)
            angle = int(float(cmd[1]))
            print(velocity)
            # To restrict values for Ackerman
            if angle > 25 or angle < -25:
                rotation_status = 1
            else:
                rotation_status = 0

            if angle > 23:
                angle = 23
                left_flag = 1
                right_flag = 0
            elif angle < -23:
                angle = -23
                left_flag = 0
                right_flag = 1

            if angle > 0:
                angle = angle
            elif angle < 0:
                angle = 360+angle
            #print("Time passed: ",time_passed)
            start = time.time()
        
        # Recovery mode by the autonomous navigation algorithm is angle == 100
        if recovery_status == 1 or rotation_status == 1:
            if left_flag == 0 and right_flag == 1:
                setdriving(1,0,6000,lrm,c,5)
                print("ROTATING RIGHT")

            elif left_flag == 1 and right_flag == 0:
                setdriving(2,0,6000,lrm,c,5)
                print("ROTATING LEFT")
        else:
            setdriving(0,angle,velocity,lrm,c,1)
            if left_flag == 1 and right_flag == 0:
                print("CORRECTING TRAJECTORY LEFT")
            if left_flag == 0 and right_flag == 1:
                print("CORRECTING TRAJECTORY RIGHT")
        
        print("velocity: ",velocity,"angle: ",angle)
        if goal_status == 3:
            print("GOAL REACHED!")
            # celebration
            lrm.pantilt.Pan = calibrated_pantilt[0]
            lrm.pantilt.Tilt = calibrated_pantilt[1]+50
            setstop(lrm,c,1)

        #format = "%(asctime)s: %(message)s"
        #logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

        #lrm_rosdata.data = lrmArray
        #pub.publish(lrm_rosdata)
        rate.sleep()

# ================= LOOP =========================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass