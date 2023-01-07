#!/usr/bin/python3
# Import the necessary libraries
from threading import Thread
import threading
from time import sleep, time
import rospy # Python library for ROS
from sensor_msgs.msg import CompressedImage # Image is the message type
from std_msgs.msg import Float32MultiArray,Int32MultiArray
import sys
import rosgraph
import numpy
import queue
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
import cv2 # OpenCV library

flag_master = 0
flag_test = 0

class SensorData():
    voltages = [0.0 for i in range(2)]
    straingauges = [0 for i in range(3)]
    ldrs = [0 for i in range(3)]
    angles = [0.0 for i in range(3)]

# thread responsible to get all data published on the ROS network and emit the good signals
class ROSRunner(QThread):
    TIMEOUT_SEC = 5
    VIDEO_WIDTH = 320
    VIDEO_HEIGHT = 240
    rosCoreConnectionChanged = Signal(bool)
    videofeedConnectionChanged = Signal(bool)
    sensorsConnectionChanged = Signal(bool)
    colorMarkerChanged = Signal(float,float,float)
    sensorDataChanged = Signal(SensorData)

    startAllVapChampLinAct = Signal(str, float) # up/down, duration
    stopAllVapChampLinAct  = Signal()

    pumpStarted = Signal() # up/down
    pumpFinished  = Signal()

    videofeedConnected = False
    rosCoreConnected = False
    sensorsConnected = False

    vapChambMovRequests = queue.Queue()
    isServingVapChambMovRequest = False
    lastRequestStopTime = None # when we should stop the current request aka stop the linear actuators
    
    isPumpRequested = False
    isServingPumpRequest = False
    lastPumpRequestStopTime = None # when we should stop the current request aka stop the pump

    class DecompressionService(QThread):
        newDecompressedImage = Signal(QImage)

        def __init__(self, cimages, *args, **kwargs):
            super(ROSRunner.DecompressionService, self).__init__(*args, **kwargs)
            self.cimageQ = cimages

        def run(self):
            global flag_test
            print("Starting Decompression Service")
            while True:
                sleep(0.1)
                if self.isInterruptionRequested():
                    return
                elif not self.cimageQ.empty():
                    convertToQtFormat = QImage() # THIS LINE SAVED MY LIFE >;) we have to clean the buffer!
                    image = self.cimageQ.get()
                    # image is compressed, decompress and then show
                    np_arr = numpy.frombuffer(image.data, numpy.uint8)
                    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    rgbImage = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgbImage.shape
                    bytesPerLine = ch * w
                    convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                    p = convertToQtFormat.scaled(VideoFeedWidget.FRAME_WIDTH, VideoFeedWidget.FRAME_HEIGHT, Qt.KeepAspectRatio)
                    self.newDecompressedImage.emit(p)

    def __init__(self, *args, **kwargs):
        super(ROSRunner, self).__init__(*args, **kwargs)
        self.cimages = queue.Queue(maxsize=1)
        self.decompressionService = self.DecompressionService(cimages=self.cimages)
        self.receiveOn = True
        self.lastVideofeed = time()
        self.lastSensorsData = time()

    def run(self):        
        subframes = rospy.Subscriber('/video_frames', CompressedImage, self._newVideoFrameCallback, queue_size=1)
        submarker = rospy.Subscriber('/color_marker_topic', Float32MultiArray, self._newColorMarkerCallback, queue_size=1)
        subsensor = rospy.Subscriber('/sensors_topic', Float32MultiArray, self._newSensorsDataCallback, queue_size=1)
        self.decompressionService.start()
        # Go through the loop 10 times per second
        rate = rospy.Rate(10)
        
        # While ROS is still running.
        while not rospy.is_shutdown():
            if self.isInterruptionRequested():
                if self.decompressionService.isRunning():
                    self.decompressionService.requestInterruption()
                return
            # Sleep just enough to maintain the desired rate
            rate.sleep()

            if not self.isServingVapChambMovRequest:
                # start movement if a request is waiting
                if not self.vapChambMovRequests.empty():
                    seconds,direction = self.vapChambMovRequests.get()

                    self.lastRequestStopTime = time() + seconds
                    self.isServingVapChambMovRequest = True
                    self.startAllVapChampLinAct.emit(direction,seconds)
                    print(f"Starting {direction} movement in vaporization chamber actuators for {seconds} seconds")
            else:
                # currently serving movement, check if we should stop
                if self.lastRequestStopTime < time():
                    self.isServingVapChambMovRequest = False
                    self.stopAllVapChampLinAct.emit()
                    print("Stopping vaporization chamber actuators")

            # pump logic
            if not self.isServingPumpRequest:
                # start movement if a request is waiting
                if self.isPumpRequested:
                    self.lastPumpRequestStopTime = time() + 20.0 # seconds
                    self.isServingPumpRequest = True
                    self.pumpStarted.emit()
                    print(f"Starting vaporization chamber pump")
                    self.isPumpRequested = False
            else:
                # currently serving pump, check if we should stop
                if self.lastPumpRequestStopTime < time():
                    self.isServingPumpRequest = False
                    self.pumpFinished.emit()
                    print("Stopping vaporization chamber pump")

            #check video feed
            if self.videofeedConnected and time() - self.lastVideofeed > self.TIMEOUT_SEC :
                self.videofeedConnected = False
                self.videofeedConnectionChanged.emit(False)

            #check sensors, show disconnected if necessary
            if self.sensorsConnected and time() - self.lastSensorsData > self.TIMEOUT_SEC:
                self.sensorsConnected = False
                self.sensorsConnectionChanged.emit(False)

            #check roscore connection
            if not self.rosCoreConnected and rosgraph.is_master_online():
                self.rosCoreConnected = True
                self.rosCoreConnectionChanged.emit(True)
            elif self.rosCoreConnected and not rosgraph.is_master_online():
                self.rosCoreConnected = False
                self.rosCoreConnectionChanged.emit(False)

    def _newVideoFrameCallback(self, image : CompressedImage):
        if not self.cimages.full() and self.receiveOn:
            self.cimages.put(image)
            # emit connection changed if necessary
            if not self.videofeedConnected:
                self.videofeedConnected = True
                self.videofeedConnectionChanged.emit(True)
        self.lastVideofeed = time()

    def _newColorMarkerCallback(self, marker : Float32MultiArray):
        # scale x and y positions
        x = marker.data[0] * float(VideoFeedWidget.FRAME_WIDTH) / float(self.VIDEO_WIDTH)
        y = marker.data[1] * float(VideoFeedWidget.FRAME_HEIGHT) / float(self.VIDEO_HEIGHT)
        self.colorMarkerChanged.emit(x,y,0)

    def _newSensorsDataCallback(self, sensors : Float32MultiArray):
        if not self.sensorsConnected:
            self.sensorsConnected = True
            self.sensorsConnectionChanged.emit(True)
        # temp1,temp2,volt1,volt2,straingauge1,straingauge2,straingauge3,ldr1,ldr2,ldr3,ldr4,ldr5,ldr6,ldr7,ldr8,heading,pitch,pm
        s = SensorData()
        s.voltages = sensors.data[0:2]
        s.straingauges = sensors.data[2:5]
        s.ldrs = sensors.data[5:9]
        s.angles = sensors.data[9:12]
        self.sensorDataChanged.emit(s)     
        self.lastSensorsData = time()

    def setReceiveVideoFeed(self, on):
        self.receiveOn = on
        if on and not self.decompressionService.isRunning():
            self.decompressionService.start()
        if not on and self.decompressionService.isRunning():
            self.decompressionService.requestInterruption()

    def durationMovementRequested(self, up, seconds):
        self.vapChambMovRequests.put((seconds, "up" if up else "down"))

    def pumpRequested(self):
        self.isPumpRequested = True

class CmdPublisher(QThread):
    CMD_WIDTH = 12
    currentCmd = [0 for _ in range(CMD_WIDTH)]

    def __init__(self, *args, **kwargs):
        super(CmdPublisher, self).__init__(*args, **kwargs)

    def run(self):
        global flag_master
        pub = rospy.Publisher('/luiee_topic', Int32MultiArray, queue_size=1)

        # Go through the loop 5 times per second
        rate = rospy.Rate(10) # 5hz

        lastCmd = None

        # While ROS is still running.
        while not rospy.is_shutdown():
            if self.isInterruptionRequested():
                return
            # Sleep just enough to maintain the desired rate
            rate.sleep()

            if self.currentCmd != lastCmd:
                ma = Int32MultiArray()
                ma.data = self.currentCmd
                pub.publish(ma)
                lastCmd = self.currentCmd[:]
            elif self.currentCmd == lastCmd and flag_master == 1:
                ma = Int32MultiArray()
                ma.data = self.currentCmd
                pub.publish(ma)
                lastCmd = self.currentCmd[:]
                flag_master = 0
    
    def setMotorPwm(self, which, pwm):
        if which < 2:
            self.currentCmd[which] = int(pwm)
        self.currentCmd[10] = 0
        self.currentCmd[11] = 0
            
    def setLEDRingState(self, state):
        self.currentCmd[2] = 5-int(state)
        self.currentCmd[10] = 0
        self.currentCmd[11] = 0

    def setBuzzer(self, pwm):
        self.currentCmd[3] = int(pwm)
        self.currentCmd[10] = 0
        self.currentCmd[11] = 0
            
    def setLinActPwm(self, which, pwm):
        if which < 3:
            self.currentCmd[4 + which] = int(pwm)
        self.currentCmd[10] = 0
        self.currentCmd[11] = 0
            
    def setStepperAngle(self, which, angle):
        if which < 3:
            self.currentCmd[7 + which] = int(angle)
        self.currentCmd[10] = 0
        self.currentCmd[11] = 0

    def setHeadingCalibration(self, flag):
        self.currentCmd[10] = 1
        self.currentCmd[11] = 0

    def setHeadingObtention(self, flag):
        self.currentCmd[10] = 0
        self.currentCmd[11] = 1

    # PENDING TO ADD THE CALIBRATION AND THE DATA OBTENTION BUTTONS FOR THE IMU
            
class MainWindow(QWidget):
    def __init__(self, rosrunner, cmdpub, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle('LUIEE GUI')
        self.setGeometry(100, 100, 1020, 680)

        self.keyboardctrlgroup = QButtonGroup()
        # different sections of the gui + connect them
        connectivity = ConnectivityWidget()
        rosrunner.videofeedConnectionChanged.connect(lambda on, w=connectivity: w.setConnected("videofeed", on))
        rosrunner.rosCoreConnectionChanged.connect(lambda on, w=connectivity: w.setConnected("roscore", on))
        rosrunner.sensorsConnectionChanged.connect(lambda on, w=connectivity: w.setConnected("sensor", on))
        
        # battery
        battery = BatteryWidget()
        for i in range(battery.NB_BAT):
            rosrunner.sensorDataChanged.connect(lambda data, b=battery, idx=i: b.setVolts(idx,data.voltages[idx]))

        # vaporization chamber
        vapchamb = VaporizationChamberWidget(keyboardRadioGroup=self.keyboardctrlgroup)
        for i in range(vapchamb.NB_LIN_ACT):
            rosrunner.sensorDataChanged.connect(lambda data, v=vapchamb, idx=i: v.setStrainGauge(idx,data.straingauges[idx]))

        rosrunner.startAllVapChampLinAct.connect(lambda direction, duration, v=vapchamb: v.setStatus(f"Moving all lin. act. {direction.upper()} for {duration}s"))
        rosrunner.startAllVapChampLinAct.connect(lambda direction, duration,v=vapchamb: v.startAll(direction))
        rosrunner.stopAllVapChampLinAct.connect(lambda v=vapchamb: v.setStatus("Standby"))
        rosrunner.stopAllVapChampLinAct.connect(lambda v=vapchamb: v.stopAll())
        rosrunner.pumpStarted.connect(lambda v=vapchamb: v.setStatus("Extracting water"))
        rosrunner.pumpFinished.connect(lambda v=vapchamb: v.setStatus("Water extracted"))
        vapchamb.durationMovementRequested.connect(rosrunner.durationMovementRequested)
        vapchamb.pumpRequested.connect(rosrunner.pumpRequested)
        vapchamb.pwmChanged.connect(cmdpub.setLinActPwm)

        # video
        videofeed = VideoFeedWidget()
        videofeed.toggled.connect(rosrunner.setReceiveVideoFeed)
        rosrunner.decompressionService.newDecompressedImage.connect(videofeed.setImage)
        rosrunner.colorMarkerChanged.connect(videofeed.setAruco)

        loco = LocomotionControlWidget(keyboardRadioGroup=self.keyboardctrlgroup)
        loco.pwmChanged.connect(lambda pair: cmdpub.setMotorPwm(1,pair[0,0]))
        loco.pwmChanged.connect(lambda pair: cmdpub.setMotorPwm(0,pair[0,1]))
        
        # Lens and mirror mechanism        
        lm = LensMirrorWidget(keyboardRadioGroup=self.keyboardctrlgroup)
        lm.angleChanged.connect(cmdpub.setStepperAngle)
        for i in range(lm.NB_ANGLES):
            rosrunner.sensorDataChanged.connect(lambda data, l=lm, idx=i: l.setReceivedAngle(idx,data.angles[idx]))

        #LDR
        ldr = LdrWidget()
        for i in range(ldr.NB_LDR):
            rosrunner.sensorDataChanged.connect(lambda data, l=ldr, idx=i: l.setLdrVal(idx,data.ldrs[idx]))
        ldr.calibrationFlagChanged.connect(lambda data=1: cmdpub.setHeadingCalibration(data))
        ldr.obtentionFlagChanged.connect(lambda data=1: cmdpub.setHeadingObtention(data))

        # general
        general = MiscWidget()
        general.roverStateChanged.connect(lambda state, g=general: cmdpub.setLEDRingState(g.roverStates.index(state)))
        general.buzzerStateChanged.connect(lambda on: cmdpub.setBuzzer( 255 if on else 0 ))

        # Reset All signal connection, connect to everything
        general.resetAll.connect(lambda v=vapchamb: v.setStatus("Standby"))
        for i in range(vapchamb.NB_LIN_ACT):
            general.resetAll.connect(lambda v=vapchamb, idx=i: v.setPwm(idx,0))
        general.resetAll.connect(lambda l=loco: l.resetPwm())
        for i in range(len(lm.axes)):
            general.resetAll.connect(lambda l=lm, idx=i: l.resetStepperAngle(idx))
        general.resetAll.connect(lambda g=general: g.resetBuzzer())
        #general.resetAll.connect(lambda g=general: g.resetLEDring())   # PENDING TO IMPLEMENT

        col1 = QVBoxLayout()
        col2 = QVBoxLayout()
        col3 = QVBoxLayout()

        col1.addWidget(connectivity)
        col1.addWidget(battery)
        col1.addWidget(vapchamb)
        col2.addWidget(videofeed)    
        col2.addWidget(loco)    
        col3.addWidget(lm)    
        col3.addWidget(ldr)  
        col3.addWidget(general)           

        c1 = QWidget()
        c2 = QWidget()
        c3 = QWidget()
        c1.setLayout(col1)
        c2.setLayout(col2)
        c3.setLayout(col3)

        h = QHBoxLayout()
        h.addWidget(c1)
        h.addWidget(c2)
        h.addWidget(c3)

        self.setLayout(h)

class KbControlRadioButton(QRadioButton):
    def __init__(self, keyboardRadioGroup, keyProcessor, text="Control with keyboard", *args, **kwargs):
        super(KbControlRadioButton, self).__init__(*args, **kwargs)
        self.setText(text)
        keyboardRadioGroup.addButton(self)
        self.keyProcessor = keyProcessor

    def keyPressEvent(self, event):
        if event.key() in [Qt.Key_Left,Qt.Key_Right] :
            event.ignore()
        elif event.key() in [Qt.Key_Up,Qt.Key_Down]:
            self.keyProcessor.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in [Qt.Key_Left,Qt.Key_Right] :
            event.ignore()
        elif event.key() in [Qt.Key_Up,Qt.Key_Down]:
            self.keyProcessor.keyReleaseEvent(event)
            
class ConnectivityWidget(QWidget):
    STRING_CONNECTED = "Connected"
    COLOR_CONNECTED = "lightgreen"
    STRING_NOT_CONNECTED = "Not connected"
    COLOR_NOT_CONNECTED = "red"


    def __init__(self, *args, **kwargs):
        super(ConnectivityWidget, self).__init__(*args, **kwargs)

        layout = QGridLayout()
        self.setLayout(layout)

        groupbox = QGroupBox("Connectivity")
        groupbox.setCheckable(False)
        layout.addWidget(groupbox)

        form = QFormLayout()
        form.setFormAlignment(Qt.AlignLeft)
        form.setLabelAlignment(Qt.AlignRight)
        groupbox.setLayout(form)

        self.indicators = {
            "roscore" : QLabel(),
            "videofeed" : QLabel(),
            "sensor" : QLabel(),
        }

        form.addRow('ROS Core:', self.indicators["roscore"])
        form.addRow('Video Feed:', self.indicators["videofeed"])
        form.addRow('Sensors:', self.indicators["sensor"])

        self.setConnected("roscore", False)
        self.setConnected("videofeed", False)
        self.setConnected("sensor", False)

        self.show()    

    def setConnected(self, name, connected):

        label = self.indicators[name]

        if connected:
            label.setText(self.STRING_CONNECTED)
            label.setStyleSheet("background-color: {}".format(self.COLOR_CONNECTED))
        else:
            label.setText(self.STRING_NOT_CONNECTED)
            label.setStyleSheet("background-color: {}".format(self.COLOR_NOT_CONNECTED))

class BatteryProgressWidget(QProgressBar):

    # voltage threshold, progress string, bar color
    bat_states = [
        (0.00, "Critical", "red"),
        (12.5, "Warning", "dark yellow"),
        (13.8, "Ok", "lightgreen")
    ]
    voltage = 0.00
    MIN_VOLTAGE = 10.0 # empirically calculated
    MAX_VOLTAGE = 14.4 # empirically calculated

    def __init__(self, *args, **kwargs):
        super(BatteryProgressWidget, self).__init__(*args, **kwargs)
        self.setTextVisible(True)
        self.setRange(0,100)
        self.setAlignment(Qt.AlignCenter) 
        self.setStyleSheet("QProgressBar { font: bold 14px; }")
        self.setBatVoltage(0)
  
    def setBatVoltage(self,value):
        # clamp voltage to min and max
        self.voltage = max(float(value),self.MIN_VOLTAGE)
        self.voltage = min(self.voltage, self.MAX_VOLTAGE)
        percent = int(100.0 * (self.voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE))
        self.setValue(percent)

        # find battery state depending on voltage
        state = next(s for s in reversed(self.bat_states) if self.voltage >= s[0])
        self.setFormat(f"{state[1]} ({percent}%)")
        self.setStyleSheet(f"QProgressBar::chunk {{ background: {state[2]}; }}")
        self.update()

class BatteryWidget(QWidget):
    valueChanged = Signal(int,float)

    NB_BAT = 2

    voltages = [0 for _ in range(NB_BAT)]

    def __init__(self, *args, **kwargs):
        super(BatteryWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)

        groupbox = QGroupBox("Power")
        groupbox.setCheckable(False)
        layout.addWidget(groupbox)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        groupbox.setLayout(form)

        for i in range(self.NB_BAT):
            hbox = QHBoxLayout()
            progress = BatteryProgressWidget()
            self.valueChanged.connect(lambda which,val,this=i,p=progress: p.setBatVoltage(float(val)) if which == this else 0)
            hbox.addWidget(progress)
            # value
            l = QLabel()
            l.setMinimumWidth(40)
            l.setAlignment(Qt.AlignCenter)
            self.valueChanged.connect(lambda which,val,this=i,label=l: label.setText(f"{float(val):.2f}") if which == this else 0)
            hbox.addWidget(l)
            #unit
            hbox.addWidget(QLabel("Volts"),alignment=Qt.AlignLeft)
            form.addRow(f"Battery {i+1}:",hbox)

            self.setVolts(0, 0)
            self.setVolts(1, 0)
        
    def setVolts(self, which, value):
        if float(value) > 16.0:
             self.voltages[which] = 16.0
        else:
            self.voltages[which] = float(value)
        self.valueChanged.emit(which,self.voltages[which])

class VaporizationChamberWidget(QWidget):

    pwmChanged = Signal(int,int)
    sgChanged = Signal(int,int) # strain gauge
    tempChanged = Signal(int,float) # temperature
    statusChanged = Signal(str) # string of status
    durationMovementRequested = Signal(bool,float) # (up=True/down=False,seconds)
    pumpRequested = Signal() # (up=True/down=False,seconds)
    NB_LIN_ACT = 3
    NB_TEMP = 2

    pwmValues = [ 0 for _ in range(NB_LIN_ACT) ]
    sgValues  = [ 0 for _ in range(NB_LIN_ACT) ]
    tempValues  = [ 0 for _ in range(NB_TEMP) ]
    status = "Standby"

    calibrated_values_up = [230,255,230]

    def __init__(self, keyboardRadioGroup, *args, **kwargs):
        super(VaporizationChamberWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)

        self.indicators = dict()
        self.ctrl       = dict()

        groupbox = QGroupBox("Vaporization Chamber")
        groupbox.setCheckable(False)
        layout.addWidget(groupbox)

        
        vbox = QVBoxLayout()
        groupbox.setLayout(vbox)

        w = QWidget()
        hbox = QHBoxLayout()
        w.setLayout(hbox)
        vbox.addWidget(w)

        for i in range(self.NB_TEMP):
            # value
            l = QLabel(f"Temperature {i+1}:")
            l.setFixedWidth(100)
            hbox.addWidget(l)

            l = QLabel("0.00 ºC")
            l.setFixedWidth(50)
            self.tempChanged.connect(lambda which,val,this=i,lab=l: lab.setText(f"{val:.2f} ºC") if which == this else 0)
            hbox.addWidget(l)
            
        #status indicator
        l = QLabel("Status:")
        l.setFixedWidth(40)
        l.setAlignment(Qt.AlignRight)
        hbox.addWidget(l)

        l = QLabel("Standby")
        l.setMinimumWidth(40)
        l.setAlignment(Qt.AlignLeft)
        self.statusChanged.connect(lambda status, lab=l: lab.setText(status))
        hbox.addWidget(l)

        masterbox = QGroupBox("Master Control")
        vbox.addWidget(masterbox)
        grid = QGridLayout()
        masterbox.setLayout(grid)
        self.ctrlAll = KbControlRadioButton(keyboardRadioGroup,self,text="Controll ALL with keyboard")
        grid.addWidget(self.ctrlAll,0,0,1,2,alignment=Qt.AlignCenter)

        #pump button
        button = QPushButton("Pump")
        button.setMinimumWidth(150)
        button.clicked.connect(lambda: self.pumpRequested.emit())
        grid.addWidget(button,0,2,1,2,alignment=Qt.AlignCenter)

        # seperate
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        grid.addWidget(line, 1,0,1,4)

        grid.addWidget(QLabel("Time-bound movement:"),2,0)
        secondsEdit = QLineEdit()
        secondsEdit.setMaximumWidth(150)
        secondsEdit.setPlaceholderText("seconds")
        grid.addWidget(secondsEdit,2,1)
        combo = QComboBox(self)
        combo.addItem("up")
        combo.addItem("down")
        grid.addWidget(combo,2,2)
        button = QPushButton("Go!")
        button.setMaximumWidth(150)
        button.clicked.connect(lambda e=secondsEdit,c=combo: self.onGo(e,c))
        grid.addWidget(button,2,3)
        
        w = QWidget()
        hbox = QHBoxLayout()
        w.setLayout(hbox)
        vbox.addWidget(w)
        
        self.sliders = [ None for _ in range(self.NB_LIN_ACT) ]
        # we have 3 linear actuators
        for i in range(3):
            linactbox = QGroupBox(f"Linear Actuator {i+1}")
            hbox.addWidget(linactbox)

            vbox = QVBoxLayout()

            l = QLabel("Current PWM",)
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))    
            vbox.addWidget(l)
            l = QLabel("0")
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))
            self.pwmChanged.connect(lambda w,val,this=i,lab=l : lab.setText(str(val)) if w == this else 0)    
            vbox.addWidget(l)

            # seperate
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            vbox.addWidget(line)

            # Lin act control
            kbcontrol = KbControlRadioButton(keyboardRadioGroup,self)
            vbox.addWidget(kbcontrol)
            upbutt = QToolButton()
            upbutt.setArrowType(Qt.UpArrow)
            upbutt.setMinimumSize(50,50)
            vbox.addWidget(upbutt, alignment=Qt.AlignCenter)
            downbutt = QToolButton()
            downbutt.setArrowType(Qt.DownArrow)
            downbutt.setMinimumSize(50,50)
            vbox.addWidget(downbutt, alignment=Qt.AlignCenter)
            vbox.addWidget(QLabel("PWM"))
            slider = QSlider(orientation=Qt.Horizontal)
            slider.setRange(0,255)
            slider.setValue(self.calibrated_values_up[i])
            slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
            valabel = QLabel(str(slider.value()))
            slider.valueChanged.connect(lambda val,l=valabel: l.setText(str(val)))
            upbutt.pressed.connect(lambda i=i,s=slider: self.setPwm(which=i,value=s.value()))
            upbutt.released.connect(lambda i=i,s=slider: self.setPwm(which=i,value=0))
            downbutt.pressed.connect(lambda i=i,s=slider: self.setPwm(which=i,value=-s.value()))
            downbutt.released.connect(lambda i=i,s=slider: self.setPwm(which=i,value=0))
            vbox.addWidget(slider)
            vbox.addWidget(valabel)
            self.sliders[i] = slider

            # seperate
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            vbox.addWidget(line)

            # Strain gauge info
            l = QLabel("Strain Gauge",)
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))    
            vbox.addWidget(l)
            l = QLabel("0%")
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))  
            self.sgChanged.connect(lambda w,val, lab=l, this=i: lab.setText(str(val)+"%") if w == this else 0)  
            vbox.addWidget(l)

            linactbox.setLayout(vbox)

            # save the control widgets
            self.ctrl["pwm"+str(i)] = (kbcontrol,slider,upbutt,downbutt)

    def keyPressEvent(self, event, pressAllOverride=False):
        if event.key() == Qt.Key_Up:
            for i,(kbcontrol,slider,upbutt,downbutt) in enumerate(self.ctrl.values()):
                if self.ctrlAll.isChecked() or pressAllOverride or kbcontrol.isChecked():
                    upbutt.setCheckable(True)
                    upbutt.setChecked(True)
                    self.setPwm(which=i, value=slider.value())
                if self.ctrlAll.isChecked() and not pressAllOverride:
                    # ctrl all with physical key
                    self.setStatus(f"Moving all linear actuators UP")
        elif event.key() == Qt.Key_Down:
            for i,(kbcontrol,slider,upbutt,downbutt) in enumerate(self.ctrl.values()):
                if self.ctrlAll.isChecked() or pressAllOverride or kbcontrol.isChecked():
                    downbutt.setCheckable(True)
                    downbutt.setChecked(True)
                    self.setPwm(which=i, value=-slider.value())
                if self.ctrlAll.isChecked() and not pressAllOverride:
                    self.setStatus(f"Moving all linear actuators DOWN")
            
    def keyReleaseEvent(self, event, pressAllOverride=False):
            if event.key() == Qt.Key_Up:
                for i,(kbcontrol,slider,upbutt,downbutt) in enumerate(self.ctrl.values()):
                    if self.ctrlAll.isChecked() or pressAllOverride or kbcontrol.isChecked():
                        upbutt.setChecked(False)
                        upbutt.setCheckable(False)
                        self.setPwm(which=i, value=0)
                        self.setStatus(f"Standby")
            elif event.key() == Qt.Key_Down:
                for i,(kbcontrol,slider,upbutt,downbutt) in enumerate(self.ctrl.values()):
                    if self.ctrlAll.isChecked() or pressAllOverride or kbcontrol.isChecked():
                        downbutt.setChecked(False)
                        downbutt.setCheckable(False)
                        self.setPwm(which=i, value=0)
                        self.setStatus(f"Standby")

    def onGo(self, edit, combo):
        seconds = 0
        try:
            seconds = float(edit.text())
            if str(combo.currentText()) == "up":
                self.durationMovementRequested.emit(True,seconds)
            else:
                self.durationMovementRequested.emit(False,seconds)
        except ValueError:
            pass
        finally:
            edit.setText("")

    def setPwm(self, which, value):
        old = self.pwmValues[which]
        self.pwmValues[which] = value
        if old != value:
            self.pwmChanged.emit(which,-value)

    def startAll(self, direction):
        # do like we pressed all the buttons at the same time 
        if direction == "up":
            self.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Up, Qt.NoModifier), pressAllOverride=True)
        elif direction == "down":
            self.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Down, Qt.NoModifier), pressAllOverride=True)

    def stopAll(self):
        # do like we released all the buttons at the same time 
        self.keyReleaseEvent(QKeyEvent(QEvent.KeyRelease, Qt.Key_Up, Qt.NoModifier), pressAllOverride=True)
        self.keyReleaseEvent(QKeyEvent(QEvent.KeyRelease, Qt.Key_Down, Qt.NoModifier), pressAllOverride=True)

    def setStrainGauge(self, which, value): # 0<value<1024
        new = int( 100.0 * float(value) / 1024.0)
        old = self.sgValues[which]
        self.sgValues[which] = new
        if abs(old - new) > 5:
            self.sgChanged.emit(which,self.sgValues[which])
        
        if all([ val > 90 for val in self.sgValues]):
            self.statusChanged.emit("Sealed")

    def setTemperature(self, which, value: float): 
        old = self.tempValues[which]
        self.tempValues[which] = value
        if abs(old - value) > 0.001:
            self.tempChanged.emit(which, self.tempValues[which])

    def setStatus(self, status: str): 
        old = self.status
        self.status = status
        if self.status != old:
            self.statusChanged.emit(self.status)

class VideoFeedWidget(QWidget):
    FRAME_WIDTH = 480
    FRAME_HEIGHT= 360
    RECT_SIDE   = 20

    toggled = Signal(bool)
    aruco = [0,0]
    def __init__(self, *args, **kwargs):
        super(VideoFeedWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)
        gbox = QGroupBox("Video Feed")
        gbox.setCheckable(True)
        gbox.toggled.connect(lambda on: self.toggled.emit(on))
        layout.addWidget(gbox)
        layout = QGridLayout()
        gbox.setLayout(layout)

        self.videolabel = QLabel()
        self.videolabel.setMinimumSize(self.FRAME_WIDTH,self.FRAME_HEIGHT)
        layout.addWidget(self.videolabel)

        self.currentFeed = QPixmap(self.FRAME_WIDTH,self.FRAME_HEIGHT)

    @Slot(QImage)
    def setImage(self, image):
        self.currentFeed = QPixmap.fromImage(image)
        self.drawAll()

    @Slot(int,int)
    def setAruco(self, x, y, z):
        self.aruco = [int(x),int(y)]
        self.drawAll()

    def drawAll(self):
        # update videofeed image
        base = QPixmap(self.currentFeed)
        pt = QPainter(base)
        if self.aruco != [0,0]:
            x,y = self.aruco
            color = QColor("orange")
            pt.setPen(QPen(color, 3, Qt.SolidLine))
            pt.drawRect(int(x) - self.RECT_SIDE/2, int(y) - self.RECT_SIDE/2, self.RECT_SIDE, self.RECT_SIDE)
        pt.end()
        self.videolabel.setPixmap(base)

class LocomotionControlWidget(QWidget):

    pwmChanged = Signal(numpy.matrix)
    keysPressed = {
        Qt.Key_Up : False,
        Qt.Key_Down : False,
        Qt.Key_Left : False,
        Qt.Key_Right : False,
    }

    def __init__(self, keyboardRadioGroup, *args, **kwargs):
        super(LocomotionControlWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)
        gbox = QGroupBox("Locomotion Control")
        layout.addWidget(gbox)
        layout = QGridLayout()
        gbox.setLayout(layout)

        self.indicators = dict()

        for i,(side,y,x) in enumerate([("L", 0,0), ("R",0,2)]):
            w = QWidget()
            wl = QVBoxLayout()
            w.setLayout(wl)
            l = QLabel(f"Current PWM {side}",)
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))    
            wl.addWidget(l)
            l = QLabel("0")
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))    
            self.pwmChanged.connect(lambda vec,l=l,i=i: l.setText(str(vec[0,i])))
            wl.addWidget(l)
            layout.addWidget(w,y,x)
            self.indicators[side] = l
        
        # seperate
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        layout.addWidget(line,1,0,1,3)

        # add keys
        self.kbcontrol = KbControlRadioButton(keyboardRadioGroup, self)
        layout.addWidget(self.kbcontrol,2,0,1,3,alignment=Qt.AlignCenter)

        self.buttons = dict()

        for key,arrow,y,x in [(Qt.Key_Up,Qt.UpArrow,3,1),
                                (Qt.Key_Down,Qt.DownArrow,5,1),
                                (Qt.Key_Left,Qt.LeftArrow,4,0),
                                (Qt.Key_Right,Qt.RightArrow,4,2)]:
            butt = QToolButton()
            butt.setArrowType(arrow)
            butt.setMinimumSize(50,50)
            butt.pressed.connect(lambda k=key: self.newKeyEvent(k, pressed=True))
            butt.released.connect(lambda k=key: self.newKeyEvent(k, pressed=False))
            layout.addWidget(butt, y,x, alignment=Qt.AlignCenter)
            self.buttons[key] = butt

        layout.addWidget(QLabel("PWM"),6,0,1,3,alignment=Qt.AlignCenter)
        self.slider = QSlider(orientation=Qt.Horizontal)
        self.slider.setRange(0,255)
        self.slider.setValue(100)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
        valabel = QLabel(str(self.slider.value()))
        self.slider.valueChanged.connect(lambda val,l=valabel: l.setText(str(val)))
        layout.addWidget(self.slider,7,0,1,3)
        layout.addWidget(valabel,8,0,1,3,alignment=Qt.AlignCenter)

        b = QPushButton("STOP EVERYTHING!")
        b.setMinimumSize(150,50)
        b.clicked.connect(self.stopAll)
        layout.addWidget(b,8,0,1,3,alignment=Qt.AlignCenter)

    def keyPressEvent(self, event):
        if not self.kbcontrol.isChecked() or event.key() not in [Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right]:
            return

        button = self.buttons[event.key()]
        button.setCheckable(True)
        button.setChecked(True)

        self.newKeyEvent(event.key(), pressed=True)

    def keyReleaseEvent(self, event):
        if not self.kbcontrol.isChecked() or event.key() not in [Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right]:
            return
        
        button = self.buttons[event.key()]
        button.setChecked(False)
        button.setCheckable(False)

        self.newKeyEvent(event.key(), pressed=False)

    def newKeyEvent(self, key, pressed):

        self.keysPressed[key] = pressed
        #recalculate vector
        vector = numpy.matrix([0.0,0.0])

        # translate pwm percentages for L and R with arrow keys
        key2pwm = {
            Qt.Key_Up    : numpy.matrix([1.0,1.0]),
            Qt.Key_Down  : numpy.matrix([-1.0,-1.0]),
            Qt.Key_Left  : numpy.matrix([0.0,1.0]),
            Qt.Key_Right : numpy.matrix([1.0,0.0]),
        }

        # direction of pwm is different if down is pressed
        if self.keysPressed[Qt.Key_Up]:
            # forward, do noting
            pass
        elif self.keysPressed[Qt.Key_Down]:
            # backward
            key2pwm[Qt.Key_Left] *= -1
            key2pwm[Qt.Key_Right] *= -1

        #add up all vectors, there might be multiple arrows pressed at the same time
        for k,v in key2pwm.items():
            if self.keysPressed[k]:
                vector += v

        norm = numpy.linalg.norm(vector)
        '''if int(norm) != 0:
            vector /= (norm/sqrt)'''
        
        # vector is normalized, multiply 
        vector *= self.slider.value()

        if vector[0,0] > 255:
            vector[0,0] = 255;
        elif vector[0,0] < -255:
            vector[0,0] = -255;
        if vector[0,1] > 255:
            vector[0,1] = 255;
        elif vector[0,1] < -255:
            vector[0,1] = -255;

        #pwm is a natural number
        self.pwmChanged.emit(vector.astype(int))

    def resetPwm(self):
        self.pwmChanged.emit(numpy.matrix([0,0]).astype(int))

    def stopAll(self):
        global flag_master
        vector = numpy.matrix([0.0,0.0])
        self.pwmChanged.emit(vector.astype(int))
        flag_master = 1

class LensMirrorWidget(QWidget):
    NB_ANGLES = 3
    angReceivedValues  = [ 0.0 for _ in range(NB_ANGLES) ]
    angleChanged = Signal(int,int)
    angleReceivedChanged = Signal(int,float)
    INCREMENT = 10
    axes = [
        "Heading",
        "Pitch",
        "Plane Mirror"
    ]

    value = [
        "Step [u]",
        "Step [u]",
        "Angle [°]"
    ]

    units = ["[°]","[°]","[stp]"]

    range_max = [200,200,360]
    intervals = [100,100,180]
    axesValues = [0,0,0]
    setValue = [0,0,0]
    sliders = list()
    kbcontrollers = list()

    def __init__(self, keyboardRadioGroup, *args, **kwargs):
        super(LensMirrorWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)
        gbox = QGroupBox("Lens and Mirror")
        layout.addWidget(gbox)
        layout = QHBoxLayout()
        gbox.setLayout(layout)

        # we have 3 linear actuators
        for i in range(3):
            lmbox = QGroupBox(f"{self.axes[i]} Control")
            layout.addWidget(lmbox)

            vbox = QVBoxLayout()
            lmbox.setLayout(vbox)

            l = QLabel("Current Value",)
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))    
            vbox.addWidget(l)
            l = QLabel("0")
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont(l.fontInfo().family(), pointSize=16))
            self.angleReceivedChanged.connect(lambda which,angle,this=i, la=l: la.setText(f"{angle:.2f} °") if which == this else 0)
            vbox.addWidget(l)

            # seperate
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            vbox.addWidget(line)

            reset = QPushButton("Reset")
            reset.pressed.connect(lambda which=i: self.resetStepperAngle(which=which))
            vbox.addWidget(reset, alignment=Qt.AlignCenter)

            kbcontrol = KbControlRadioButton(keyboardRadioGroup,self)
            vbox.addWidget(kbcontrol)

            dualbuttonw = QWidget()
            hbox = QHBoxLayout()
            dualbuttonw.setLayout(hbox)
            vbox.addWidget(dualbuttonw)

            leftbutt = QToolButton()
            leftbutt.setArrowType(Qt.LeftArrow)
            leftbutt.setMinimumSize(50,50)
            hbox.addWidget(leftbutt,alignment=Qt.AlignCenter)
            rightbutt = QToolButton()
            rightbutt.setArrowType(Qt.RightArrow)
            rightbutt.setMinimumSize(50,50)
            hbox.addWidget(rightbutt,alignment=Qt.AlignCenter)
            self.kbcontrollers.append((kbcontrol,leftbutt,rightbutt))

            vbox.addWidget(QLabel(f"{self.value[i]} Increment"))
            slider = QSlider(orientation=Qt.Horizontal)
            slider.setRange(0,self.range_max[i])
            slider.setValue(self.setValue[i])
            slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
            slider.setTickInterval(self.intervals[i])
            self.sliders.append(slider)
            valabel = QLabel(f"{self.setValue[i]}")
            slider.valueChanged.connect(lambda val,l=valabel: l.setText(str(val)))
            leftbutt.pressed.connect(lambda this=i: self.incrementStepperAngle(this, reverse=True))
            rightbutt.pressed.connect(lambda this=i: self.incrementStepperAngle(this, reverse=False))
            vbox.addWidget(slider)
            vbox.addWidget(valabel)

    def keyPressEvent(self, event):
        keypressed = event.key() 
        if keypressed not in [Qt.Key_Left,Qt.Key_Right]:
            return

        reverse = (keypressed == Qt.Key_Left)

        for i,(ctrl,l,r) in enumerate(self.kbcontrollers):
            if ctrl.isChecked():
                if reverse:
                    button = l
                else:
                    button = r
                button.setCheckable(True)
                button.setChecked(True)
                self.incrementStepperAngle(i, reverse)

    def keyReleaseEvent(self, event):
        keyreleased = event.key() 
        if keyreleased not in [Qt.Key_Left,Qt.Key_Right]:
            return
        reverse = (keyreleased == Qt.Key_Left)

        for i,(ctrl,l,r) in enumerate(self.kbcontrollers):
            if ctrl.isChecked():
                if reverse:
                    button = l
                else:
                    button = r
                button.setChecked(False)
                button.setCheckable(False)

    def resetStepperAngle(self,which):
        global flag_master
        self.axesValues[which] = 0
        self.angleChanged.emit(which, self.axesValues[which])

    def incrementStepperAngle(self, which, reverse):
        global flag_master
        #update new angle
        if not reverse:
            self.axesValues[which] = self.sliders[which].value()
        else:
            self.axesValues[which] = -self.sliders[which].value()

        self.angleChanged.emit(which, self.axesValues[which])
        flag_master = 1

    def setReceivedAngle(self, which, value: float):
        new = value
        old = self.angReceivedValues[which]
        self.angReceivedValues[which] = new
        if abs(old - new) > 1 and which != 2:
            self.angleReceivedChanged.emit(which,self.angReceivedValues[which])
        elif which == 2:
            self.angleReceivedChanged.emit(which,self.angReceivedValues[which])

class LdrWidget(QWidget):
    WIDTH = 300
    NB_LDR = 4
    LDR_RADIUS = 30
    CENTER2CIRCLE = 105

    calibrationFlagChanged = Signal()
    obtentionFlagChanged = Signal()

    valueChanged = Signal(int,int)

    ldrvalues = [0 for _ in range(NB_LDR)]

    def __init__(self, *args, **kwargs):
        super(LdrWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)
        gbox = QGroupBox("LDR Positioning")
        gbox.setMinimumSize(self.WIDTH,self.WIDTH)
        layout.addWidget(gbox)
        layout = QHBoxLayout()
        gbox.setLayout(layout)

        self.label = QLabel()
        self.label.setMinimumSize(self.WIDTH,self.WIDTH)
        layout.addWidget(self.label, alignment=Qt.AlignCenter)

        b = QPushButton("Calibrate Azimuth")
        b.setMaximumWidth(150)
        b.clicked.connect(self.Azimuth_calibration)
        layout.addWidget(b,alignment=Qt.AlignCenter)

        b = QPushButton("Obtain Azimuth")
        b.setMaximumWidth(150)
        b.clicked.connect(self.Azimuth_obtention)
        layout.addWidget(b,alignment=Qt.AlignCenter)

    def Azimuth_calibration(self):
        global flag_master
        self.calibrationFlagChanged.emit()
        flag_master = 1

    def Azimuth_obtention(self):
        global flag_master
        self.obtentionFlagChanged.emit()
        flag_master = 1

    def paintEvent(self, event):
        pixmap = QPixmap(self.WIDTH,self.WIDTH)
        pixmap.fill(QColor("lightgray"))

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPoint(self.WIDTH / 2, self.WIDTH / 2)

        # draw circles
        for i,val in enumerate(self.ldrvalues):
            angle = i * (2.0 * numpy.pi / self.NB_LDR) + numpy.pi
            color = QColor("black")
            painter.setPen(QPen(color, 3, Qt.SolidLine))
            color = QColor("yellow")
            color.setAlpha(255 * float(val) / 1024.0)
            painter.setBrush(QBrush(color, Qt.SolidPattern))
            painter.drawEllipse(center.x() + self.CENTER2CIRCLE * numpy.sin(angle) - self.LDR_RADIUS,
                                center.y() + self.CENTER2CIRCLE * numpy.cos(angle) - self.LDR_RADIUS, 
                                2*self.LDR_RADIUS, 
                                2*self.LDR_RADIUS)
            painter.drawText(center.x() + self.CENTER2CIRCLE * numpy.sin(angle) - .5*self.LDR_RADIUS,
                             center.y() + self.CENTER2CIRCLE * numpy.cos(angle) - .5*self.LDR_RADIUS,
                             f"LDR {i+1}")
            color = QColor("black")
            color.setAlpha(150)
            painter.setPen(QPen(color, 3, Qt.SolidLine))
            painter.drawText(center.x() + self.CENTER2CIRCLE * numpy.sin(angle) - .2*self.LDR_RADIUS,
                             center.y() + self.CENTER2CIRCLE * numpy.cos(angle) - .1*self.LDR_RADIUS,
                             f"{int(100.0*val/1024.0)}%",)
        painter.end()

        self.label.setPixmap(pixmap)

    def resizeEvent(self,event):
        super(LdrWidget, self).resizeEvent(event)
        
        ratio = float(event.size().height()) / float(event.oldSize().height())
        if ratio > 0:
            self.CENTER2CIRCLE *= ratio
            self.LDR_RADIUS *= ratio
            self.update()


    def setLdrVal(self, which, value):
        new = value
        old = self.ldrvalues[which]
        self.ldrvalues[which] = new
        if new == 1:#abs(old - new) > 2000:
            self.valueChanged.emit(which, int(value))
            self.update()

class MiscWidget(QWidget):

    resetAll = Signal()
    roverStateChanged = Signal(str) # state number
    buzzerStateChanged = Signal(bool) #on/off

    roverStates = [
        "Navigation Mode",
        "Extraction Mode",
        "Storage Mode",
        "Receding Mode",
        "Control Mode",
    ]

    def __init__(self, *args, **kwargs):
        super(MiscWidget, self).__init__(*args, **kwargs)
        layout = QGridLayout()
        self.setLayout(layout)
        gbox = QGroupBox("General")
        layout.addWidget(gbox)
        hbox = QHBoxLayout()
        gbox.setLayout(hbox)

        c = QComboBox()
        c.addItems([ state for state in self.roverStates ])
        hbox.addWidget(c)

        b = QPushButton("Set LED ring")
        b.pressed.connect(lambda combo=c: self.roverStateChanged.emit(combo.currentText()))
        hbox.addWidget(b)

        # seperate
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        # hbox.addWidget(line)

        self.cb = QCheckBox("Buzzer")
        self.cb.setChecked(False)
        self.cb.stateChanged.connect(lambda dummy,c=self.cb: self.buzzerStateChanged.emit(c.isChecked()))
        hbox.addWidget(self.cb,alignment=Qt.AlignCenter)

        # seperate
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        # hbox.addWidget(line)

        b = QPushButton("Reset All")
        b.setMinimumSize(150,50)
        b.clicked.connect(lambda: self.resetAll.emit())
        hbox.addWidget(b)

    #def resetLEDring(self):
    #self.cb.setChecked(False)

    def resetBuzzer(self):
        self.cb.setChecked(False)

def wai(b):
    sleep(2)
    b.setVolts(1,14)
    sleep(2)
    b.setVolts(1,12.4)
    print("Finish")
  
def main():
    # initialize GUI elements
    app = QApplication(sys.argv)
    rospy.init_node('gui_py', anonymous=True)

    runner = ROSRunner()
    publisher = CmdPublisher()

    window = MainWindow(rosrunner=runner,cmdpub=publisher)
    window.showMaximized()
    #window.show()
    runner.start()
    publisher.start()
    sys.exit(app.exec())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        pass
