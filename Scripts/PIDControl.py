#!/usr/bin/env python3
import rospy
import math as m
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

Kp = 31.1029;
Ki = 59.2506;
Kd = 4.0817;

integration = 0
e_k1        = 0
veal        = 0

publisher   = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

def quaternion_to_Euler(w,x,y,z):
    t0 = 2*(w*x + y*z)
    t1 = 1 - 2*(x*x + y*y)
    X  = m.atan2(t0,t1)
    return X

def callback(msg):
    w     = msg.orientation.w
    x     = msg.orientation.x
    y     = msg.orientation.y
    z     = msg.orientation.z
    angle = quaternion_to_Euler(w,x,y,z)
    control(-angle)

def control(angle):
    global integration,e_k1,veal
    setPoint = 0
    error    = setPoint - angle
    integration += error*0.01
    controlAction = Kp*error + Ki*integration + Kd*(error - e_k1)/0.01
    e_k1 = error
    #control Action è una forza 
    c = veal + (controlAction/0.6)*0.01
    veal = c
    command  = Twist()
    command.linear.x = c
    publisher.publish(command)



if __name__ == "__main__":
    rospy.init_node("Imu_Sensor_Reader")
    sub = rospy.Subscriber("/imu",Imu,callback)
    rospy.spin()
