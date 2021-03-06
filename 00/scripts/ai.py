#!/usr/bin/env python
import rospy
import json
import numpy
import numpy.linalg as LA
import sys
import math
from std_msgs.msg import String
from soccer.srv import *

class AINode:

    # kalmanFilter, pathPrediction, controlSystem, comNode

    def __init__(self):
        self.role = "offense"
        self.state = "start"
        self.l = 3.048
        self.w = 1.52
        self.kvx = 5
        self.kvy = 5
        self.kphi = 2
        self.oppGoal = numpy.array([self.l/2, 0])
        self.markers = numpy.array([[-self.l/2,0,self.l/2,self.l/2,0,-self.l/2],[self.w/2,self.w/2,self.w/2,-self.w/2,-self.w/2,-self.w/2]])
        # pillar1 = self.markers[0,:]
        self.robot = numpy.array([0,0,0]) #[x,y,r]
        self.ball = numpy.array([0,0,0,0]) #[x,y,vx,vy]
        self.ballVision = [0,0]
        self.goalVision = [0,0]
        self.centerOfGoal = [self.l/2,0]
        self.stationary = [0,0,0]
        self.ir = False
        self.v1 = [0,0,0]

        rospy.init_node('ai', anonymous=False)
        # Subscribers
        rospy.Subscriber("commAiRefChatter", String, self.refCallback)
        rospy.Subscriber("commAiTeamChatter", String, self.teamCallback)
        rospy.Subscriber("kalmanAiChatter", String, self.kalmanCallback)
        rospy.Subscriber("visionAiChatter", String, self.visionCallback)
        rospy.Subscriber("pathAiChatter", String, self.pathCallback)
        rospy.Subscriber("controlAiChatter", String, self.controlCallback)
        rospy.Subscriber("irAiChatter", String, self.irCallback)
        # Publishers
        self.teamPub = rospy.Publisher("aiCommTeamChatter", String)
        self.debugPub = rospy.Publisher("aiCommDebugChatter", String)
        self.pathPub = rospy.Publisher("aiPathChatter", String)
        self.wheelPub = rospy.Publisher("aiControlWheelsChatter", String)
        self.kickerPub = rospy.Publisher("aiControlKickerChatter", String)
        self.kalmanPub = rospy.Publisher("aiKalmanChatter", String)
        self.dribblerPub = rospy.Publisher("aiControlDribblerChatter", String)
        self.r = rospy.Rate(10)
        self.Rmatrix = numpy.zeros(shape=(3,3))
        self.Rmatrix[0] = [math.cos(self.robot[2]), math.sin(self.robot[2]), 0]
        self.Rmatrix[1] = [-math.sin(self.robot[2]), math.cos(self.robot[2]), 0]
        self.Rmatrix[2] = [0, 0, 1]
        print "Finished initialization"
        
    def run(self):
        while not rospy.is_shutdown():
            if self.state == "start":
                if self.role == "offense":
                    if not self.ir and self.ballVision[1] > abs(.5):
                        self.skillRotate("right", 0.2)
                    elif not self.ir:
                        self.playRushBall()
                    else:
                        self.rotateToGoalAndShoot()
                    '''
                    if numpy.count_nonzero(self.ball) == 0:
                        self.rotate()
                    else:
                        self.playRushGoal()
                        '''
                else:
                    if numpy.count_nonzero(self.robot) == 0:
                        self.skillGoToPoint(self.centerOfGoal)
                    else:
                        self.skillStayInGoal()

                '''
                wheel_vel = {}
                wheel_vel["velocities"] = self.Rmatrix.dot(self.v1).tolist()
                '''

                vel = {}
                vel["velocities"] = self.v1
                #vel["velocities"] = self.stationary
                self.wheelPub.publish(json.dumps(vel))
                self.kalmanPub.publish(json.dumps(vel))
                self.r.sleep()

    def useDribbler(self, use):
        dribble = {}
        dribble["on"] = use
        self.dribblerPub.publish(json.dumps(dribble))

    def playRushBall(self):
        self.useDribbler(True)
        if self.ballVision[1] > .05:
            self.skillRotate("right", 0.1)
        elif self.ballVision[1] < -.05:
            self.skillRotate("left", 0.1)
        else:
            self.v1 = [0,0.7,0] # Go forward?

    def rotateToGoal(self):
        self.useDribbler(True)
        while abs(self.goalVision[1]) > .05:
            if self.goalVision[1] > .05:
                self.skillRotate("right", 0.1)
            elif self.goalVision[1] < -.05:
                self.skillRotate("left", 0.1)
        self.shoot()

    def shoot():
        self.useDribbler(False)
        kick = {}
        kick["kick"] = True
        self.kickerPub.publish(json.dumps(kick))
        self.v1 = self.stationary

    def playRushGoal(self):
        self.debugPub.publish("Running: Play > RushGoal")
        n = self.oppGoal-self.ball[0:2]
        n = n/LA.norm(n)

        position = self.ball[0:2] - 0.2*n

        if LA.norm(position-self.robot[0:2])<.21:
            self.skillGoToPoint(self.oppGoal)
        else:
            self.skillGoToPoint(position)

    def skillRotate(self, direction, speed):
        self.debugPub.publish("Running: Skill > Rotate")
        if direction == "right":
            direction = int(-1)
        else:
            direction = int(0)
        self.v1 = [0,0,direction*speed]
        rospy.loginfo("Rotating")

    def skillGoToPoint(self, position):
        self.debugPub.publish("Running: Skill > GoToPoint")
        vx = -self.kvx*(self.robot[0]-position[0])
        vy = -self.kvy*(self.robot[1]-position[1])
        theta = math.atan2(self.ball[1]-self.robot[1], self.ball[0]/2+.1-self.robot[0])
        omega = -self.kphi*(self.robot[2] - theta)
        self.v1 = [vx,vy,omega]

    def skillStayInGoal(self):
        self.debugPub.publish("Running: Skill > StayInGoal")
        x_pos = -self.l/2
        threshold = .18
        if self.ball[1]>threshold:
            ball_y = threshold
        elif self.ball[1]<-threshold:
            ball_y = -threshold
        else:
            ball_y = self.ball[1]

        vx = -self.kvx*(self.robot[0]-x_pos)
        vy = -self.kvy*(self.robot[1]-ball_y)
        theta_d = math.atan2(self.ball[1]-self.robot[1], self.ball[0]/2+.1-self.robot[0])
        omega = -self.kphi*(self.robot[2]-theta_d)
        self.v1 = [vx,vy,omega]
            
    def refCallback(self, data):
        self.debugPub.publish("got Ref data")
        data = json.loads(data.data)
        command = data["ref"]

        def stop():
            self.state = "stop"
        def start():
            self.state = "start"
        def reset():
            self.state = "reset"
            # TODO: Set a better start position
            self.skillGoToPoint([0,0])
  
        commands = {'0': stop, '1': start, '2': reset}

        commands[command]()

    def teamCallback(self, data):
        data = json.loads(data.data)
        self.robot = numpy.array(data["self"])
        self.ball = numpy.array(data["ball"])
        # TODO: Something else? This is team data
        self.debugPub.publish("got Teammate data")

    def kalmanCallback(self, data):
        data = json.loads(data.data)
        self.selfState = data["self"]
        self.ballState = data["ball"]
        rospy.loginfo("got Kalman Data, self:%s, ball:%s", self.selfState, self.ballState)
        self.debugPub.publish("got Kalman data")

    def visionCallback(self, data):
        rospy.loginfo(data.data)
        data = json.loads(data.data)
        self.ballVision = [data["ball"]["distance"], data["ball"]["angle"]]
        self.goalVision = [data["goal"]["distance"], data["goal"]["angle"]]
        self.debugPub.publish("got Kalman data")

    def pathCallback(self, data):
        data = json.loads(data.data)
        # TODO

    def controlCallback(self, data):
        data = json.loads(data.data)
        # TODO

    def irCallback(self, data):
        data = json.loads(data.data)
        if data["value"] == 1:
            self.ir = True
        else:
            self.ir = False

if __name__ == "__main__":
    ai = AINode()
    ai.run()
