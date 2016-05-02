#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info
import random


# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        self.fields = []

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
            twist = Twist()
            deltaX = 0
            deltaY = 0
            for field in self.fields:
                if field.id <= 1 and field.calcVelocity(msg)[0] == 0 and field.calcVelocity(msg)[1]==0:
                    deltaX = 0
                    deltaY = 0
                    break
                else:
                    deltaX += field.calcVelocity(msg)[0]
                    deltaY += field.calcVelocity(msg)[1]
            # Change twist.linear.x to be your desired x velocity
            print deltaX

            twist.linear.x = deltaX
            # Change twist.linear.y to be your desired y velocity
            twist.linear.y = deltaY
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmdVelPub.publish(twist)

    def start(self):
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]
                x = 0.0;
                y = 0.0;
                for p in poly.points:
                    x = x + int(p.x)
                    y = y + int(p.y)
                x = x/len(poly.points)
                y = y/len(poly.points)
                r = math.sqrt(math.pow(int(poly.points[0].x)-x,2) + math.pow(int(poly.points[0].y)-y,2))
                s = r
                fieldType = False
                if t_id == 0:
                    self.fields.append(AttractiveField(t_id,x,y,.6,s,r))
                elif t_id == 1:
                    self.fields.append(TangentialField(t_id,x,y,.6,s,r))
                else:
                    self.fields.append(RepulsiveField(t_id,x,y,.5,2*s,r))
            self.fields.append(RandomField(0,0,0,0,0))
        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False

    def stop(self):
        self.stop = True


class Field:
    def __init__(self,id, xpos, ypos, alpha, s, r):
        self.id = id
        self.xpos = xpos
        self.ypos = ypos
        self.alpha = alpha
        self.s = s
        self.r = r

    def calculateDistance(self,x0,x1,y0,y1):
        distance = math.sqrt(math.pow(int(x0)-x1,2) + math.pow(int(y0)-y1,2))
        return distance

class AttractiveField(Field):
    def calcVelocity(self, msg):
        result = [0,0]
        distance = self.calculateDistance(int(msg.x), self.xpos, int(msg.y), self.ypos)
        theta = math.atan2(-self.ypos + int(msg.y),self.xpos - int(msg.x))
        if distance < self.r:
            return result
        if self.r<=distance<=(self.s + self.r):
            result[0] = (self.alpha * (distance - self.r) * math.cos(theta))
            result[1] = (self.alpha * (distance - self.r) * math.sin(theta))
        elif distance > (self.s + self.r):
            result[0] = (self.alpha * self.s * math.cos(theta))
            result[1] = (self.alpha * self.s * math.sin(theta))
        return result

class RepulsiveField(Field):
    def calcVelocity(self, msg):
        result = [0,0]
        distance = self.calculateDistance(int(msg.x), self.xpos, int(msg.y), self.ypos)
        theta = math.atan2(-self.ypos + int(msg.y),self.xpos - int(msg.x))
        if distance < self.r:
            result[0] = -math.cos(theta) * 1000
            result[1] = -math.sin(theta) * 1000
            return result
        if self.r<=distance<=(self.s + self.r):
            result[0] = -self.alpha * (self.s + self.r - distance) * math.cos(theta)
            result[1] = -self.alpha * (self.s + self.r - distance) * math.sin(theta)
        elif distance > (self.s + self.r):
            result[0] = 0
            result[1] = 0
        return result

class TangentialField(Field):
    def calcVelocity(self, msg):
        result = [0,0]
        distance = self.calculateDistance(int(msg.x), self.xpos, int(msg.y), self.ypos)
        theta = math.atan2(-self.ypos + int(msg.y),self.xpos - int(msg.x))
        if distance < self.r:
            return result
        elif distance > (self.s + self.r):
            print "outside of s field"
            result[0] = (self.alpha * self.s * math.cos(theta))
            result[1] = (self.alpha * self.s * math.sin(theta))
        elif math.degrees(theta)<=0 or math.degrees(theta)>=180:
            print "inside of s field in tangential area."
            # result[0] = -(self.s + self.r - distance) * math.cos(theta + math.pi/2)
            result[0] = -(distance - self.r) * math.cos(theta + math.pi/2)
            result[1] = -(distance - self.r) * math.sin(theta + math.pi/2)
            # result[0] += (self.alpha/4 * (distance - self.r) * math.cos(theta))
            # result[1] += (self.alpha/4 * (distance - self.r) * math.sin(theta))
        else:
            print "inside of s field and out of tangential area"
            result[0] = (self.alpha * (distance - self.r) * math.cos(theta))
            result[1] = (self.alpha * (distance - self.r) * math.sin(theta))
        return result

class RandomField(Field):
    def calcVelocity(self,msg):
        result = [0,0]
        result[0] = random.randint(-5,5)
        result[1] = random.randint(-5,5)
        return result

class SpheroIntrudeForm(QtGui.QWidget):
    controller = Controller()
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480) 
        self.initUI()

        rospy.init_node('sphero_intrude', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback) 

        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback) 
       
    def initUI(self):

        self.stateLabel = QtGui.QLabel("Position")
        self.stateTextbox = QtGui.QTextEdit()
        self.stateTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), self.updateStateTextbot)     
        
        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)  
        self.connect(self, QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.aprilTagsInfoLabel = QtGui.QLabel("april tags info")
        self.aprilTagsInfoBtn = QtGui.QPushButton("Query")
        self.aprilTagsInfoBtn.clicked.connect(self.queryAprilTagsInfo)
        self.aprilTagsTextbox = QtGui.QTextEdit()
        self.aprilTagsTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), self.updateAprilTagsTextbot)

        self.aprilTagsStartBtn = QtGui.QPushButton("Start")
        self.aprilTagsStartBtn.clicked.connect(self.controller.start)

        self.aprilTagsStopBtn = QtGui.QPushButton("Stop")
        self.aprilTagsStopBtn.clicked.connect(self.controller.stop)


        self.layout =  QtGui.QVBoxLayout()
        self.layout.addWidget(self.stateLabel)
        self.layout.addWidget(self.stateTextbox)
        self.layout.addWidget(self.keyInstructLabel)
        self.layout.addWidget(self.cmdVelLabel)
        self.layout.addWidget(self.cmdVelTextbox)
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.aprilTagsInfoLabel)
        hlayout.addWidget(self.aprilTagsInfoBtn)
        hlayout.addWidget(self.aprilTagsStartBtn)
        hlayout.addWidget(self.aprilTagsStopBtn)
        self.layout.addLayout(hlayout)
        self.layout.addWidget(self.aprilTagsTextbox)
        self.setLayout(self.layout)

        self.setWindowTitle("Sphero Intrude")
        self.show()

    def keyPressEvent(self, e): 
        twist = None       
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()  
            twist.linear.x = 0; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
        if twist != None:
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def trackposCallback(self, msg):
        rospy.wait_for_service("apriltags_intrude")
        try:
            intrude_query = rospy.ServiceProxy("apriltags_intrude", apriltags_intrude)
            resp = intrude_query(int(msg.x), int(msg.y))
            pos_id_text = "["+str(int(msg.x))+"," +str(int(msg.y))+"]" + "(" + str(resp.id) + ")"
            self.emit(QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), pos_id_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def updateStateTextbot(self, value):
        self.stateTextbox.moveCursor(QtGui.QTextCursor.End)
        self.stateTextbox.ensureCursorVisible()
        self.stateTextbox.append(str(value))
        self.stateTextbox.update()

    def queryAprilTagsInfo(self):
        #print "clicked"
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()
               
            #print str(resp)

            info_text = "" 
            for i in range(len(resp.polygons)):
                poly = resp.polygons[i]
                t_id = resp.ids[i]

                #print(str(poly))
                #print(str(t_id))
                info_text += "["+str(t_id)+"] "
                for p in poly.points:
                    info_text += "(" + str(int(p.x)) + "," + str(int(p.y)) + ")"
                info_text += "\n" 

            self.emit(QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), info_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateAprilTagsTextbot(self, value):
        self.aprilTagsTextbox.clear()
        self.aprilTagsTextbox.moveCursor(QtGui.QTextCursor.End)
        self.aprilTagsTextbox.ensureCursorVisible()
        self.aprilTagsTextbox.append(str(value))
        self.aprilTagsTextbox.update()        


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroIntrudeForm()
    w.show()
    sys.exit(app.exec_())
  
        
