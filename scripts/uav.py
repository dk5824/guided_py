#!/usr/bin/env python3

import rospy
import mavros
import time, math
import numpy as np

from mavros_msgs.msg import *
from mavros_msgs.srv import * #CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import * # PoseSatmped, TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from geographic_msgs.msg import GeoPoseStamped

class Uav:
    def __init__(self):

        #define publisher
        self.local_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.global_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        #define subscriber
        rospy.Subscriber("/mavros/state", State, self.stateCallback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.localPositionCallback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.globalPositionCallback)
        rospy.Subscriber("/mavros/battery", BatteryState, self.batteryCallback)

        #define service client
        self.arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.change_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.land_service = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

        #define object need to subscribing several topic
        self.current_local_position = PoseStamped()
        self.current_global_position = NavSatFix()
        self.current_state = State()
        self.current_battery_state = BatteryState()


    #define callback functions
    def stateCallback(self, data):
        self.current_state = data

    def localPositionCallback(self, data):
        self.current_local_position.pose = data.pose
    

    def globalPositionCallback(self, data):
        self.current_global_position = data

    def batteryCallback(self, data):
        self.current_battery_state = data

    #arm vehicle
    def arming(self):
        print("\n----------armingCall----------")
        rospy.wait_for_service("mavros/cmd/arming")
        out = self.arm(True)
        rospy.loginfo(out)
        rospy.sleep(3)

    #disarm vehicle
    def disarming(self):
        print("\n--------disarmingCall---------")
        rospy.wait_for_service("mavros/cmd/arming")
        out = self.arm(False)
        rospy.loginfo(out)
        rospy.sleep(3)

    #change the vehicle's mode
    def switch_modes(self, current_mode, next_mode):
        print("\n----------switch_modes----------")
        rospy.wait_for_service("/mavros/set_mode")
        out = self.change_mode(current_mode, next_mode)
        rospy.loginfo(out)

    def takeoff(self, height):
        print("\n----------takeoff_call----------")
        out = self.takeoff_service(0, 0, 0, 0, height)
        rospy.loginfo(out)
        rospy.sleep(3)

    def land(self):
        print("\n----------land_call----------")
        out = self.land_service(0,0,0,0,0)
        rospy.loginfo(out)

    #move to local setpoint
    def local_servoing(self, x, y, z):
        print("servoing to ", x, y, z)

        positopub = PoseStamped()
        positopub.header.stamp = rospy.Time.now()

        positopub.pose.position.x =  x
        positopub.pose.position.y =  y
        positopub.pose.position.z =  z
        
        #to correct heading angle as current angle
        positopub.pose.orientation.x = self.current_local_position.pose.orientation.x
        positopub.pose.orientation.y = self.current_local_position.pose.orientation.y
        positopub.pose.orientation.z = self.current_local_position.pose.orientation.z
        positopub.pose.orientation.w = self.current_local_position.pose.orientation.w
        
        self.local_pub.publish(positopub)
        
    #move to global setpoint(ECEF frame)    
    def global_servoing(self, lati, longi, alti): #move to goal global position

        print("servoing to lat,  lon, alt", lati, longi, alti)

        positopub = GeoPoseStamped()
        positopub.header.stamp = rospy.Time.now()
     
        positopub.pose.position.latitude = lati
        positopub.pose.position.longitude = longi
        positopub.pose.position.altitude = alti
     
        self.global_pub.publish(positopub)
    
    
     #todo
     #velocity