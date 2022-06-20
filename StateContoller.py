#!/usr/bin/env python3
from sre_parse import State
import rospy as rp
import numpy as np

#topic messages
from geometry_msgs.msg import Twist
from smr_pkg.msg import TwoWheelsState

class StateController():

    def __init__(self):
        #load value from parameters server
        self.M  = rp.get_param('/robot/M')                                                                   #mass pendulum
        self.mc = rp.get_param('/robot/mc')                                                                  #mass cart
        self.l  = rp.get_param('/robot/l')                                                                   #CG distance
        self.g  = rp.get_param('/robot/g')                                                                   #gravity
        self.Ts = rp.get_param('/robot/Ts')                                                                  #sampling Time
        #feeback matrix , covariance ,and output vector
        self.V1 = np.array(rp.get_param("/covariance/V1"))                                                   #state covariance matrix
        self.V2 = rp.get_param("/covariance/V2")                                                             #exit variance
        self.C  = np.array(rp.get_param("/robot/C"))                                                         #exit vector
        self.K  = np.array(rp.get_param("/control/K"))                                                       #feedback matrix
        self.x0 = np.array(rp.get_param("/robot/x0"))                                                        #initial conditions
        #init rospy node
        rp.init_node("StateController")
    
    #setup method
    def setup(self):
        #declare a publisher 
        self.publisher  = rp.Publisher("/cmd_vel",Twist,queue_size=1)
        #declare a subscriber and start control
        self.velocity_k1= 0                                                                                 #memory for velocity
        self.subscriber = rp.Subscriber("/stateExtimation",TwoWheelsState,self.controlRobot)                #loop
        rp.spin()

    #controlRobot
    def controlRobot(self,sE):
        xk               = [sE.x1 , sE.x2 , sE.x3 , sE.x4]                                                  #state extimated loading from topic
        controlAction    = self.K[0]*xk[0] + self.K[1]*xk[1] + self.K[2]*xk[2] + self.K[3]*xk[3]            #control action (Force)
        velocity_k       = self.velocity_k1 + (controlAction / (self.mc)) * self.Ts                         #approximating derivaty
        self.velocity_k1 = velocity_k
        command          = Twist()
        command.linear.x = velocity_k
        #send command
        self.publisher.publish(command)


#main method
if __name__ == '__main__':
    #start control
    SC = StateController()
    #setup method
    SC.setup()





    


