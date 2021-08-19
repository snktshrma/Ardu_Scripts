#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy
import math, time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Global parameters
x_cor = 10
y_cor = 0
yaw_in_radians = 0
yaw_rate = 0
flag = 0

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def _init_(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 10)
            print("Hello")
        except rospy.ServiceException as e:
            print("Takeoff failed.")

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Arming failed.")

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Disarming failed.")

    def setguidedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='GUIDED')
        except rospy.ServiceException as e:
            print("guided Mode could not be set.")


    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='LAND')
        except rospy.ServiceException as e:
               print("Land Mode could not be set.")

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask =  PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW_RATE
        
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 10.0
        
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.
        self.theta = np.linspace(0, 2*np.pi, 5)
        modes = fcuModes()

# the radius of the circle
        self.r = 10

        # compute x1 and x2
        self.x1 = self.r*np.cos(self.theta)
        self.x2 = self.r*np.sin(self.theta)
        self.sp.position.x = 0
        self.sp.position.y = 0
        self.abc = 0

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        global x_cor,y_cor,yaw_rate,flag
        try:
            d = 0.5
            if -d <= self.threshold(self.local_pos.x,self.sp.position.x) <= d and -d <= self.threshold(self.local_pos.y,self.sp.position.y) <= d:
                self.sp.position.x = self.x1[self.abc]
                self.sp.position.y = self.x2[self.abc]
                print(self.abc)
                self.abc += 1

            # Second Coordinate of Square
            #if self.abc == 4 and -d <= self.threshold(self.local_pos.x,self.sp.position.x) <= d and -d <= self.threshold(self.local_pos.y,self.sp.position.y) <= d:
             #   modes.setAutoLandMode()
            
        except:
            pass                
        
    def threshold(self,x,y):
        return abs(x-y)

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y - 5
n1 = 0
n2 = 0
n3 = 0

# Main function
def main():
    global n1, n2
    # initiate node
    rospy.init_node('square', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed

    modes.setguidedMode()

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    time.sleep(5)

    # set in takeoff mode and takeoff to default altitude (3 m)
    if cnt.state.armed:
        modes.setTakeoff()
        time.sleep(9)
    # We need to send few setpoint messages, then activate guided mode, to take effec

    # activate guided mode
    


    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSp()
        sp_pub.publish(cnt.sp)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

