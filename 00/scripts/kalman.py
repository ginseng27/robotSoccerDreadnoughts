#!/usr/bin/env python 

import numpy
import signal
import numpy.linalg as LA
import math
import subprocess
import json
import rospy
import time
from std_msgs.msg import String
import os

class KalmanFilter():
    def __init__(self, visionProg="/home/root/vision/goalBall", configfile="/home/root/vision/hsvValues.config"):
        l = 3.048
        w = 1.52
        pi = 3.14156
        self.S0_self = numpy.diag([l*10/12, w*10/12, pi*10/2])
        self.Q_self = numpy.diag([10000,10000,10000])
        self.R_range = .1
        self.R_bearing = pi/180
        self.S0_ball = numpy.diag([1,1,1,1,1])
        self.Q_ball = numpy.diag([1,1,10,10,.1])
        self.state_self = numpy.array([0,0,pi/2]) #this is the robot's predicted location [x,y,rotation]
        self.state_S_self = self.S0_self
        self.state_ball = numpy.array([0,0,0,0,.3]) #this is the ball's predicted location [x,y,xvel?, yvel?, mu?]
        self.state_S_ball = self.S0_ball
       
        self.markers = numpy.array([[-l/2,0,l/2,l/2,0,-l/2],[w/2,w/2,w/2,-w/2,-w/2,-w/2]])
        self.control_sample_rate = .5  #check this again?
        self.visionProg = visionProg
        self.configfile = configfile
        self.vision ={'marker' : [['NULL', 'NULL', 'NULL', 'NULL', 'NULL', 'NULL'], ['NULL', 'NULL', 'NULL', 'NULL', 'NULL', 'NULL']], 'ball' : ['NULL', 'NULL']}
        #create listener and publisher
        rospy.init_node('kalman', anonymous=False)
        #self.pub = rospy.Publisher('kalmanAiChatter', String)
        self.pub = rospy.Publisher('visionAiChatter', String)
        #rospy.Subscriber('aiKalmanChatter', String, self.utility_observer)

    def start_vision(self, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False, background=True):
        #print "runing vision in background?: %s" % background
        cmd = [self.visionProg, self.configfile]
        for x in range(0,2):
            self.visionProcess = subprocess.Popen(cmd, shell=shell, stdout=stdout, stderr=stderr)
            self.visionpid = self.visionProcess.pid
            self.pid = self.visionpid
            time.sleep(5)
            os.kill(self.visionpid, signal.SIGQUIT)
        self.visionProcess = subprocess.Popen(cmd, shell=shell, stdout=stdout, stderr=stderr)
        self.visionpid = self.visionProcess.pid
        self.pid = self.visionpid
        done = False
        print "entering while not done loop for vision"
        while not done:
            line = self.visionProcess.stdout.readline()
            if "NULL" in line:
                done = True
                break
            else:
                try:
                    float(line)
                    done = True
                except:
                    done = False
        #print "vision started. pid: %s" % self.visionpid


    def printAll(self):
        print list(self.state_ball)

    def print_self(self):
        #print "Current robot position"
        #print self.state_self
        #print "Current ball position"
        #print self.state_ball
        #print self.S0_self
        #print self.Q_self
        #print self.R_range
        #print self.R_bearing
        #print self.S0_ball
        #print self.Q_ball
        #print self.markers
        return

    def utility_observer(self, data, reset_flag=0):
        #v1 = numpy.array([0,0,0]) 
        velocities = json.loads(data.data)["velocities"]
        v1 = numpy.array([velocities[0],velocities[1],velocities[2]])
        print v1
        
        if reset_flag == 1:
            self.state_self = numpy.array([-3.048/6,0,0])
            self.state_S_self = S0_self
            self.state_ball = numpy.array([0,0,0,0,.3])
            self.state_S_ball = S0_ball
        #----------------------
        # estimate self position and heading
        self.utility_observer_self_update(
            state_self=self.state_self,
            state_S_self=self.state_S_self,
            v1=v1)
        #----------------------
        # estimate ball position and velocity
        self.utility_observer_ball_update(
            state_ball=self.state_ball, 
            state_S_ball=self.state_S_ball, 
            state_self=self.state_self,
            state_S_self=self.state_S_self)


    def utility_observer_self_update(self, state_self, state_S_self, v1):

        # estimate between measurements
        N = 10
        for i in range(0,N):
            f = v1
            state_self = state_self + (self.control_sample_rate / N) * f
            state_S_self = state_S_self + (self.control_sample_rate / N) * (self.Q_self)

        # measurement updates
        for i in range(0,6):
            # range measurement
            if self.vision['marker'][0][i] != 'NULL': 
                rho = self.markers[:, i] - state_self[0:2]
                Rho = LA.norm(rho)
                if Rho > 0.05:            # this is a hack, but avoids dividing by zero when Rho is small
                    h = Rho
                    C = numpy.array([-rho[0], -rho[1], 0]) / Rho
                    L = numpy.squeeze(numpy.asarray(state_S_self.dot(C.transpose()) / (self.R_range + C.dot(state_S_self).dot(C.transpose()))))
                    state_S_self = (numpy.diag([1,1,1]) - numpy.asmatrix(L).transpose()*C)*(state_S_self)
                    state_self = state_self + L*(self.vision['marker'][0][i] - h)

            # bearing measurement
            if self.vision['marker'][1][i] != 'NULL':
                rho = self.markers[:, i] - state_self[0:2]
                Rho = LA.norm(rho)
                if Rho > 0.05:            # this is a hack, but avoids dividing by zero when Rho is small
                    phi = state_self[2]
                    h = math.asin((rho[1] * math.cos(phi) - rho[0] * math.sin(phi)) / Rho)
                    C = numpy.sign(rho[0] * math.cos(phi) + rho[1] * math.sin(phi)) * numpy.array([rho[1] / (Rho ** 2), -rho[0] / (Rho ** 2), -1])
                    L = numpy.squeeze(numpy.asarray(state_S_self.dot(C.transpose()) / (self.R_bearing + C.dot(state_S_self).dot(C.transpose()))))
                    state_S_self = (numpy.diag([1,1,1]) - numpy.asmatrix(L).transpose()*C)*(state_S_self)
                    state_self = state_self + L*(self.vision['marker'][1][i] - h)

        self.state_self = numpy.squeeze(state_self)
        self.state_S_self = state_S_self

    def utility_observer_ball_update(self,state_ball, state_S_ball, state_self, state_S_self):
        # re-initialize ball if valid vision data, and ball_hat is behind robot
        if numpy.logical_and(((state_ball[0:2] - state_self[0:2]).transpose().dot(numpy.array([math.cos(state_self[2]), math.sin(state_self[2])])) < 0), (numpy.logical_or((self.vision['ball'][0] != 'NULL'), (self.vision['ball'][1] != 'NULL')))):
            state_ball[0:2] = state_self[0:2] + self.vision['ball'][0]*numpy.array([math.cos(state_self[2] + self.vision['ball'][1]), math.sin(state_self[2] + self.vision['ball'][1])])
            rho = state_ball[0:2] - state_self[0:2]
            Rho = LA.norm(rho)
            phi = state_self[2]
            beta = math.asin((rho[1] * math.cos(phi) - rho[0] * math.sin(phi)) / Rho)
            R = numpy.array([[math.cos(2 * beta), -math.sin(2 * beta)], [math.sin(2 * beta), math.cos(2 * beta)]])
            state_ball[2:4] = R.dot(state_ball[2:4])
            state_S_ball = numpy.diag([1, 1, 1, 1, 1])

        # estimate between measurements
        N = 10
        for i in range(0,N):
            # right hand side of differential equation for ball
            # doesn't include bounces off wall (or self)
            f = numpy.array([state_ball[2],state_ball[3], -state_ball[4] * state_ball[2],-state_ball[4]*state_ball[3],  0]).transpose()
            state_ball = state_ball + (self.control_sample_rate / N)*f
            A = numpy.array([[0,0,1,0,0],
                             [0,0,0,1,0],
                             [0,0,-state_ball[4],0,-state_ball[2]],
                             [0,0,0,-state_ball[4],-state_ball[3]],
                             [0,0,0,0,0]])
            state_S_ball = state_S_ball + (self.control_sample_rate / N) * (A.dot(state_S_ball) + state_S_ball.dot(A.transpose()) + self.Q_ball) #check this???

        # measurement updates
        # range measurement
        if self.vision['ball'][0] != 'NULL':
            rho = state_ball[0:2] - state_self[0:2]
            Rho = LA.norm(rho)
            if Rho > .05:        # this is a hack, but avoids dividing by zero when Rho is small
                h = Rho
                C = numpy.array([rho[0], rho[1], 0, 0, 0]) / Rho
                L = numpy.squeeze(numpy.asarray(state_S_ball.dot(C.transpose()) / (self.R_range + C.dot(state_S_ball).dot(C.transpose()))))
                state_S_ball = (numpy.diag([1,1,1,1,1]) - numpy.asmatrix(L).transpose()*C)*(state_S_ball)
                state_ball = state_ball + L * (self.vision['ball'][0] - h)

        # bearing measurement
        if self.vision['ball'][1] != 'NULL':
            rho = state_ball[0:2] - state_self[0:2]
            Rho = LA.norm(rho)
            if Rho > .05:        # this is a hack, but avoids dividing by zero when Rho is small
                phi = state_self[2]
                h = math.asin((rho[1] * math.cos(phi) - rho[0] * math.sin(phi)) / Rho)
                C = numpy.sign(rho[0] * math.cos(phi) + rho[1] * math.sin(phi)) * numpy.array([(-rho[1] / (Rho ** 2)), (rho[0] / (Rho ** 2)), 0, 0, 0])
                L = numpy.squeeze(numpy.asarray(state_S_ball.dot(C.transpose()) / (self.R_bearing + C.dot(state_S_ball).dot(C.transpose()))))
                state_S_ball = (numpy.diag([1,1,1,1,1]) - numpy.asmatrix(L).transpose()*C)*(state_S_ball)
                state_ball = state_ball + L * (self.vision['ball'][1] - h)

        self.state_ball = state_ball
        self.state_S_ball = state_S_ball

    def run(self):
        r = rospy.Rate(2)
        rospy.loginfo("Vision about to start")
        self.start_vision()
        rospy.loginfo("Vision started")
        while not rospy.is_shutdown():
            line = self.visionProcess.stdout.readline()
            rospy.loginfo(">>>>>>> %s", line)
            data = json.loads(line)
            #vision_temp["marker"] = [
            #        [data["pillar1"]["distance"],
            #        data["pillar2"]["distance"],
            #        data["pillar3"]["distance"],
            #        data["pillar4"]["distance"],
            #        data["pillar5"]["distance"],
            #        data["pillar6"]["distance"]],
            #        [data["pillar1"]["angle"],
            #        data["pillar2"]["angle"],
            #        data["pillar3"]["angle"],
            #        data["pillar4"]["angle"],
            #        data["pillar5"]["angle"],
            #        data["pillar6"]["angle"]]]
            #vision_temp["ball"] = [data["ball"]["distance"],
            #    data["ball"]["angle"]]


            #self.vision = vision_temp
            #data_temp = {}
            #data_temp["self"] = self.state_self[0:3].tolist()
            #data_temp["ball"] = self.state_ball[0:4].tolist()
            #data = json.dumps(data_temp)
#            rospy.loginfo(data)
           # self.pub.publish(data)
            self.pub.publish(json.dumps(data))

            r.sleep()
            
    #def listener(self):
        #filter = KalmanFilter()
        #rospy.init_node('visionSystem', anonymous=True)  
        #rospy.spin()
        
    #def talker(self):
        
        #rospy.init_node('visionSystem', anonymous=True)
        #r = rospy.Rate(60) #60 hz
        #while not rospy.is_shutdown():
        #    data_temp["self"] = self.state_self[0:3]
        #    data_temp["ball"] = self.state_ball[0:4]
        #    data = json.dumps(data_temp)
        #    #rospy.loginfo(data)
        #    pub.publish(data)
        #    r.sleep()
            

if __name__ == "__main__":
    try:
        k = KalmanFilter()
        print "About to call run()"
        k.run()    
    except rospy.ROSInterruptException: pass
