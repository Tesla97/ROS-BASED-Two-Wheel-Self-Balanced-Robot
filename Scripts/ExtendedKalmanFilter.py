#!/usr/bin/env python3
import rospy as rp
import numpy as np

#custom message
from smr_pkg.msg import TwoWheelsState

#imu message sensor
from sensor_msgs.msg import  Imu

# param definitions
# param[0] -> M (Kg)
# param[1] -> mc(Kg)
# param[2] -> l (m)
# param[3] -> g (m/s^2)
# param[4] -> Ts(sec)
# param[5] -> V1
# param[6] -> V2
# param[7] -> K
# param[8] -> x0
# param[9] -> C


class ExtendedKalmanFilter():

    #constructor
    def __init__(self):
        #to superclass
        super().__init__()
        #load paramaters from Parameter Server
        M  = rp.get_param('/robot/M')                                                                   #mass pendulum
        mc = rp.get_param('/robot/mc')                                                                  #mass cart
        l  = rp.get_param('/robot/l')                                                                   #CG distance
        g  = rp.get_param('/robot/g')                                                                   #gravity
        Ts = rp.get_param('/robot/Ts')                                                                  #sampling Time
        #feeback matrix , covariance ,and output vector
        V1 = np.array(rp.get_param("/covariance/V1"))                                                   #state covariance matrix
        V2 = rp.get_param("/covariance/V2")                                                             #exit variance
        C  = np.array(rp.get_param("/robot/C"))                                                         #exit vector
        K  = np.array(rp.get_param("/control/K"))                                                       #feedback matrix
        x0 = np.array(rp.get_param("/robot/x0"))                                                        #initial conditions
        p  = [M,mc,l,g,Ts,V1,V2,K,x0,C]                                                                 #param list
        #load parameters 
        self.param = p
    
    #setup method 
    def setup(self):
        #init rospy node
        rp.init_node("StateExtimator")
        #define a publisher
        self.topicName = "/stateExtimation"
        self.publisher  = rp.Publisher(self.topicName,TwoWheelsState,queue_size=1)
        #init Filter parameters
        self.xk   = np.array([0.0,0.0,0.0,0.0])                                                                #correction
        self.xkk1 = self.param[8]                                                                              #prediction  -> init to initial state of the system
        self.P    = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,0.1]])        #variance matrix error prediction (Riccati solution)
        self.Ko   = self.evaluateGain()                                                                        #filter gain
        self.I    = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,0.1]])        #identity matrix
        self.Kok1 = np.array([0.0,0.0,0.0,0.0])                                                                #gain at time k-1
        #define a subscriber to Imu sensors
        self.subscriber = rp.Subscriber("/imu",Imu,self.stateExtimation)                                       #loop
        rp.spin()
    
    #return topic name
    def getTopicName(self):
        return self.topicName
    
    #stateExtimation
    def stateExtimation(self,sensorMsg):
        C = self.param[9]
        P = self.P
        #convert quaternion to Euler Angle (roll - angle)
        y_misurata = self.quaternionToEuler(sensorMsg)
        #evaluate innovation
        innovation = y_misurata - self.xkk1[0]
        #evaluate filter gain
        distance   = np.sqrt((self.Ko[0] - self.Kok1[0])**2 + (self.Ko[1] - self.Kok1[1])**2 + (self.Ko[2] - self.Kok1[2])**2 + (self.Ko[3] - self.Kok1[3])**2)
        #control steady - state condition
        if(distance > 0.01):
            #measurement update
            #Po    = (self.I-self.Ko*C)*P
            Po   = (self.I - self.Ko @ C) @ P @ (self.I - self.Ko @ (C.transpose())) + (self.Ko @ self.Ko.transpose())*(self.param[6])
            #evaluate Jacobian
            J    = self.evaluateJacobian(self.xkk1[0],self.xkk1[1],self.xkk1[2],self.xkk1[3])
            #Riccati
            self.P    = J @ Po @ J.transpose() + self.param[5] 
            #update gain filter
            self.Kok1 = self.Ko
            self.Ko   = self.evaluateGain()
        #correction & prediction
        self.xk  = self.xkk1 + (self.Ko * innovation)
        #state publishing
        state    = TwoWheelsState()
        state.x1 = self.xk[0]
        state.x2 = self.xk[1]
        state.x3 = self.xk[2]
        state.x4 = self.xk[3]
        self.publisher.publish(state)
        #prediction
        self.xkk1= self.funzioneGeneratrice(self.xk[0],self.xk[1],self.xk[2],self.xk[3],self.param[0],self.param[1],self.param[2],self.param[3],self.param[7],self.param[4])


    #definizione funzione generatrice
    def funzioneGeneratrice(self,x1,x2,x3,x4,M,m,l,g,K,Ts):
        u = K[0]*x1+K[1]*x2+K[2]*x3+K[3]*x4
        #(m*l*l - ((m*m*l*l)/(M+m))*cos(x(1,k))*cos(x(1,k)));
        c = (m*(l**2) - ((m*m*l*l)/(M+m))*np.cos(x1))*np.cos(x1)
        d = (M+m - ((m*m*l**2)/(m*l**2))*np.cos(x1)**2)
        a = x2 + (m*g*l*np.sin(x1) - ((m*m*l**2)/(M+m))*np.sin(x1)*np.cos(x1)*x2*x2 + ((m*l)/(M+m))*u*np.cos(x1))*(Ts/c)
        b = x4 + (m*l*np.sin(x1)*x2**2 - ((m*m*l*l*g)/(m*l**2))*np.cos(x1)*np.sin(x1) + u)*(Ts/d)
        xp= [(x1 + x2*Ts),a,(x3 + x4*Ts),b]
        return xp


    #evaluate Gain Filter
    def evaluateGain(self):
        C  = self.param[9]                   
        A  = C @ self.P @ C.transpose() + self.param[6]
        Ko = ( self.P @ C.transpose()) * (1/A)
        return Ko


    #quaternion to Euler (Roll Angle)
    def quaternionToEuler(self,msg):
        w     = msg.orientation.w
        x     = msg.orientation.x
        y     = msg.orientation.y
        z     = msg.orientation.z
        t0    = 2*(w*x + y*z)
        t1    = 1 - 2*(x*x + y*y)
        X     = np.arctan2(t0,t1)
        return -X
 
    #evaluate Jacobian
    def evaluateJacobian(self,x1,x2,x3,x4):
        F_2_1 = ((1020425430369950873*np.cos(x1))/595935302254592000 + (2*x2**2*np.cos(x1)**2)/271 - (2*x2**2*np.sin(x1)**2)/271 - np.sin(x1)*((12840303695053645*x1)/4767482418036736 + (2862163177882475*x2)/4767482418036736 - (29120478301424165*x4)/152559437377175552 - (20*1.41*x3)/271))/(100*((2*np.cos(x1)**2)/271 - 1/20)) + (np.cos(x1)*np.sin(x1)*(np.cos(x1)*((12840303695053645*x1)/4767482418036736 + (2862163177882475*x2)/4767482418036736 - (29120478301424165*x4)/152559437377175552 - (20*1.41*x3)/271) - (981*np.sin(x1))/1000 + (2*x2**2*np.cos(x1)*np.sin(x1))/271))/(6775*((2*np.cos(x1)**2)/271 - 1/20)**2)
        F_2_2 = ((2862163177882475*np.cos(x1))/4767482418036736 + (4*x2*np.cos(x1)*np.sin(x1))/271)/(100*((2*np.cos(x1)**2)/271 - 1/20)) + 1
        F_2_3 =-(1.41*np.cos(x1))/(1355*((2*np.cos(x1)**2)/271 - 1/20))
        F_2_4 = -(5824095660284833*np.cos(x1))/(3051188747543511040*((2*np.cos(x1)**2)/271 - 1/20))
        F_4_1 = (np.cos(x1)*np.sin(x1)*((36.49*x1) + (8.13*x2) - (2.58*x4) - ((x2**2)*np.sin(x1))*0.1 + (1.96*np.cos(x1)*np.sin(x1)) - 1.41*x3))/(250*(np.cos(x1)**(2/5) - 1.35)**2) - (((x2**2)*np.cos(x1))*0.1 - (1.96*np.cos(x1)**2) + (1.96*np.sin(x1)**2) - 36.49)/(100*(np.cos(x1)**(2/5) - 1.35))
        F_4_2 = -((x2*np.sin(x1))/5 - 8.13)/(100*(np.cos(x1)**(2/5) - 1.36))
        F_4_3 = -1.41/(100*((np.cos(x1)**(2/5)) - 1.36))
        F_4_4 = 1 - 5824095660284833/(225179981368524800*(np.cos(x1)**(2/5) - 1.36))

        F     = np.array([[1,self.param[4],0,0],[F_2_1,F_2_2,F_2_3,F_2_4],[0,0,1,self.param[4]],[F_4_1,F_4_2,F_4_3,F_4_4]])

        return F

if __name__ == '__main__':
    #init EKF
    ekf = ExtendedKalmanFilter()
    #invoke setup method
    ekf.setup()




    
