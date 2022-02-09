#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy
import math, time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Global parameters
x_cor = 10
y_cor = 0
yaw_in_radians = 0
yaw_rate = 0
flag = True
fl2 = True
newx = 0.0
newy = 0.0

obs_staticx = 0.0
obs_staticy = 0.0
obs_staticyaw = 0.0

goal_y = 11
goal_x = 11

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
            print("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='GUIDED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set."%e)

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask =  PositionTarget.IGNORE_VZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW_RATE
        
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        self.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = [0.0, 0.0]

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 5.0
        
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.yaw = 1.571
        self.sp.velocity.x = 1
        self.sp.velocity.y = 1

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks
        self.theta = np.linspace(0, 2*np.pi, 100)

# the radius of the circle
        self.r = np.sqrt(100)

        # compute x1 and x2
        self.x1 = self.r*np.cos(self.theta)
        self.x2 = self.r*np.sin(self.theta)
        self.abc = 0

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def obs(self, msg):
        self.data[0]= msg.ranges[0]
        self.data[1]= msg.ranges[90]
        self.data[2]= msg.ranges[180]
        self.data[3]= msg.ranges[45]
        self.data[4]= msg.ranges[135]


    ## Update setpoint message





    
    def updateSp(self):
        global x_cor,y_cor,yaw_rate,flag, obs_staticy, obs_staticx, obs_staticyaw, fl2, newx, newy
        yaw_chk = 1.571 +  math.atan((goal_x-self.local_pos.x)/(goal_y-self.local_pos.y))
        
        self.sp.velocity.x = 1
        self.sp.velocity.y = 1
        


        if self.data[1] <= 3 and goal_y < self.data[1]:
            self.sp.velocity.x = 1
            self.sp.velocity.y = 1
            self.sp.position.y = goal_y
            #self.sp.position.x = goal_x
        
        elif goal_y > 3:
            if self.data[1] > 3:
                if flag:
                    self.sp.velocity.x = 1
                    self.sp.velocity.y = 1
                    self.sp.position.y = goal_y
                    self.sp.position.x = self.local_pos.x + 1
                    if self.local_pos.y == goal_y:
                        flag = False 
                else:
                    self.sp.velocity.x = 1
                    self.sp.velocity.y = 1
                    self.sp.yaw = yaw_chk
                    self.sp.position.y = goal_y
                    self.sp.position.x = goal_x
                self.pose[0] = self.local_pos.x
                self.pose[1] = self.local_pos.y
                self.pose[1] = self.local_pos.y + 0.8
            elif 0.8 <= self.data[1] <= 3:
                self.sp.velocity.x = 1
                self.sp.velocity.y = 1
                self.sp.position.y = self.pose[1]
                if self.data[0] >= 3:
                    if self.local_pos.x - 0.5 <= self.sp.position.x <= self.local_pos.x + 0.5:
                        self.sp.position.x += 1
                        flag = True
                elif self.data[2] >= 3:
                    if self.local_pos.x - 0.5 <= self.sp.position.x <= self.local_pos.x + 0.5:
                        self.sp.position.x -= 1
                        flag = True








        
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
    rospy.Subscriber('/scan', LaserScan, cnt.obs)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed

    modes.setOffboardMode()

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    time.sleep(5)

    # set in takeoff mode and takeoff to default altitude (3 m)
    if cnt.state.armed:
        modes.setTakeoff()
        time.sleep(9)
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effec

    # activate OFFBOARD mode
    


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


'''
print(self.sp.position.x)
        print(self.sp.position.y)
        print("_________________________")
        if self.data[1] <= 3:
            if flag:
                print("flag")
                obs_staticy = 8
                obs_staticx = 8
                obs_staticyaw = yaw_chk
                self.sp.position.y = obs_staticy
                self.sp.position.x = obs_staticy
                self.sp.yaw = obs_staticyaw
                flag = False
                fl2 = True
            elif fl2 and not obs_staticy - 0.2 <= self.local_pos.y <= obs_staticy + 0.2 and not obs_staticx - 0.2 <= self.local_pos.x <= obs_staticx + 0.2:
                print("f2")
                self.sp.position.y = obs_staticy
                self.sp.position.x = obs_staticy
                self.sp.yaw = obs_staticyaw
                

            elif obs_staticy - 0.2 <= self.local_pos.y <= obs_staticy + 0.2 and not obs_staticx - 0.2 <= self.local_pos.x <= obs_staticx + 0.2:
                fl2 = False

        elif self.data[1] <= 4 and not fl2:
            print("ddta")
            newx = (goal_y - obs_staticy)
            newy = -(goal_x - obs_staticx)
            if self.local_pos.yaw 
            self.local_pos.yaw = 1.571 +  math.atan((newx-self.local_pos.x)/(newy-self.local_pos.y))
            self.sp.velocity.x = 1
            self.sp.velocity.y = 1
            self.sp.position.y = newy
            self.sp.position.x = newx
            #flag = True

            

        elif self.data[1] > 3:
            self.sp.position.y = goal_y
            self.sp.position.x = goal_x
'''