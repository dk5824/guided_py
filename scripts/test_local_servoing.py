#!/usr/bin/env python3

from uav import Uav
import rospy

rospy.init_node('uav_node', anonymous=False)
r = rospy.Rate(5) # set loop period

uav = Uav()

x_setpoint = 3
y_setpoint = 6
height_setpoint = 5

uav.switch_modes(0,'GUIDED') #set flight mode as 'GUIDED'
rospy.sleep(1)

uav.arming() #arming vehicle using ROS Service
rospy.sleep(2)

uav.takeoff(height_setpoint) #takeoff using ROS Service

#waiting for reaching the target height
while uav.current_local_position.pose.position.z < height_setpoint - 0.1 :
    print("waiting for reaching target height: ",  uav.current_local_position.pose.position.z)
    r.sleep()

uav.local_servoing(x_setpoint, y_setpoint, height_setpoint)
rospy.sleep(2)
rospy.loginfo("end the servoing process!!")

uav.land() #landing using ROS Service