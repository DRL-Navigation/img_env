#!/usr/bin/env python
import roslib; roslib.load_manifest('collvoid_controller')
import rospy
import commands
import string
try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program"

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from std_srvs.srv import Empty
from collvoid_msgs.msg import PoseTwistWithCovariance

import tf

class controller(wx.Frame):
    
    
    def __init__(self,parent,id,title):
        wx.Frame.__init__(self,parent,id,title)
        self.parent = parent
        self.initialized = False
        self.initialize()
        
    def initialize(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)

        self.pub = rospy.Publisher('/commands_robot', String, queue_size=10)

        self.robotList = []
        self.robotList.append("all")

        self.reset_srv = rospy.ServiceProxy('/reset_positions', Empty)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Controls"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
         
        start = wx.Button(self,wx.ID_ANY,label="Start!")
        static_sizer.Add(start, 0)
        self.Bind(wx.EVT_BUTTON, self.start, start)
        
        stop = wx.Button(self,wx.ID_ANY,label="Stop!")
        static_sizer.Add(stop, 0)
        self.Bind(wx.EVT_BUTTON, self.stop, stop)

        reset = wx.Button(self,wx.ID_ANY,label="Reset!")
        static_sizer.Add(reset, 0)
        self.Bind(wx.EVT_BUTTON, self.reset, reset)


        grid_sizer = wx.GridBagSizer()
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "New Goals"), wx.HORIZONTAL)
        static_sizer.Add(grid_sizer, 0)
        sizer.Add(static_sizer, 0)
 
        self.choiceBox = wx.Choice(self,wx.ID_ANY,choices=self.robotList)

        grid_sizer.Add(self.choiceBox,(0,0),(1,2),wx.EXPAND)
        self.SetPosition(wx.Point(200,200))
        self.SetSize(wx.Size(600,200))


        setCircling = wx.Button(self,wx.ID_ANY,label="Circling On/Off")
        grid_sizer.Add(setCircling, (3,1))
        self.Bind(wx.EVT_BUTTON, self.setCircling, setCircling)


        setOnOff = wx.Button(self,wx.ID_ANY,label="Set On/Off")
        grid_sizer.Add(setOnOff, (3,0))
        self.Bind(wx.EVT_BUTTON, self.setOnOff, setOnOff)

        
        sendDelayedGoal = wx.Button(self,wx.ID_ANY,label="Send delayed Goal")
        grid_sizer.Add(sendDelayedGoal, (4,0))
        self.Bind(wx.EVT_BUTTON, self.sendDelayedGoal, sendDelayedGoal)

        self.delayTime = wx.TextCtrl(self,wx.ID_ANY,value=u"0.0")
        grid_sizer.Add(self.delayTime, (4,1))
        
        sendInitGuess = wx.Button(self,wx.ID_ANY,label="Send init Guess")
        grid_sizer.Add(sendInitGuess, (5,0))
        self.Bind(wx.EVT_BUTTON, self.sendInitGuess, sendInitGuess)

        sendNextGoal = wx.Button(self,wx.ID_ANY,label="Send next Goal")
        grid_sizer.Add(sendNextGoal, (5,1))
        self.Bind(wx.EVT_BUTTON, self.sendNextGoal, sendNextGoal)

        toggleActiveCollvoid = wx.Button(self,wx.ID_ANY,label="Toggle Active collvoid")
        grid_sizer.Add(toggleActiveCollvoid, (5,2))
        self.Bind(wx.EVT_BUTTON, self.toggleServices, toggleActiveCollvoid)

        grid_sizer.AddGrowableCol(0)
        self.SetSizer(sizer)

        self.Layout()
        self.Fit()
        self.Show(True)

        self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cbCommonPositions)
        self.initialized = True
        self.services = []

    def toggleServices(self, event):
        for s in self.services:
            try:
                s()
            except rospy.ServiceException as e:
                rospy.logwarn("could not call toggle service collviod %s", e)
        return

    def cbCommonPositions(self,msg):
        if not self.initialized:
            return
        if self.robotList.count(msg.robot_id) == 0:
            rospy.loginfo("robot added")
            self.robotList.append(msg.robot_id)
            if msg.controlled:
                s = rospy.ServiceProxy(msg.robot_id + '/toggle_active_collvoid', Empty)
                self.services.append(s)
            self.choiceBox.Append(msg.robot_id)

    def sendDelayedGoal(self, event):
        sleepTime = float(self.delayTime.GetValue())
        rospy.sleep(sleepTime)
        string = "%s send delayed Goal"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def setCircling(self,event):
        string = "%s circle"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def setOnOff(self,event):
        string = "%s WP change"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

            
    def sendNextGoal(self,event):
        string = "%s next Goal"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendInitGuess(self,event):
        string = "%s init Guess"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def stop(self,event):
        string = "%s Stop"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def start(self,event):
        string = "%s next Goal"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def all_start(self,event):
        string = "all Start"
        self.pub.publish(str(string))

    def all_init_guess(self,event):
        string = "all init Guess"
        self.pub.publish(str(string))

        
    def reset(self,event):
        self.pub.publish("all Stop")
        rospy.sleep(0.2)
        self.pub.publish("all Restart")
        #rospy.sleep(0.2)
        self.reset_srv()
            
    
if __name__ == '__main__':
    rospy.init_node('controller')
    app = wx.App()
    frame = controller(None,wx.ID_ANY,'Controller')

    app.MainLoop()
