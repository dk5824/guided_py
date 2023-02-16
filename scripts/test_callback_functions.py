#!/usr/bin/env python3

from uav import Uav
import rospy

rospy.init_node('MAVROS_Subscriber', anonymous=False)
r = rospy.Rate(1) # set loop period
uav = Uav()


while not rospy.is_shutdown():
    print("\ncurrent_local_position: \n", uav.current_local_position.pose)
    r.sleep()
    
    print("\ncurrent_global_position: \n", uav.current_global_position)
    r.sleep()
    
    print("\ncurrent_vehicle_state: \n", uav.current_state)
    r.sleep()