#!/usr/bin/env python 

import subprocess
import json
import os
import signal
import rospy
import time
from std_msgs.msg import String

class vision():
    def __init__(self, visionProg="/home/root/vision/goalBall", hsvFile="/home/root/vision/hsvValues.config"):
        rospy.init_node('VisionNode', anonymous=False)
        self.pub = rospy.Publisher('visionKalmanChatter', String)
        self.visionProg = visionProg
        self.hsvFile = hsvFile
         
    def start_vision(self, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False):
        #print "runing vision in background?: %s" % background
        cmd = [self.visionProg, self.hsvFile]
        self.visionProcess = subprocess.Popen(cmd, shell=shell, stdout=stdout, stderr=stderr)
        self.visionpid = self.visionProcess.pid
        done = False
        while not done:
            line = self.visionProcess.stdout.readline()
            try:
                json.loads(line)
                done = True
            except:
                done = False
        rospy.loginfo("cmd: %s started" % cmd)


    def restart_vision(self):
        rospy.loginfo("restarting vision")
        os.kill(self.visionpid, signal.SIGKILL)
        rospy.loginfo("killed")
        time.sleep(5)
        self.start_vision()
                    
    def run(self):
        r = rospy.Rate(2) #60 hz
        NULLcount = 0
        self.start_vision()
        while True:
            line = self.visionProcess.stdout.readline()
            print "line is: %s" % line
            data = json.loads(line)
            if "NULL" in str(data["ball"]["angle"]):
                NULLcount+=1
                print "null count: %s" % NULLcount
            elif data["ball"]["angle"] < 0:
                print "ball angle < 0 "
                NULLcount = 0
                self.restart_vision()
            elif data["ball"]["angle"] > 0:
                print "ball angle > 0"
                break

            if NULLcount >= 10:
                NULLcount = 0
                self.restart_vision()




        while not rospy.is_shutdown():
            line = self.visionProcess.stdout.readline()
            data = json.loads(line)
            self.pub.publish(json.dumps(data))
            r.sleep()
            
if __name__ == "__main__":
    try:
        k = vision()
        k.run()    
    except rospy.ROSInterruptException: pass
