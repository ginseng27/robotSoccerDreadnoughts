#!/usr/bin/env python
from std_msgs.msg import String
import rospy
import json
import subprocess

class IrReader:
    def __init__(self, gpio="250"):
        print "initializing for ir reading on gpio %s" % gpio
        self.filepath = "/sys/class/gpio/gpio250/"
        setup = ["/home/root/catkin_ws/src/soccer/scripts/irctrl/setup_gpio.sh"]
        self.execute_command(setup)
        rospy.init_node('IrNode', anonymous=False)
        self.aiPub = rospy.Publisher("irAiChatter", String)
        self.rate = rospy.Rate(5)

    def execute_command(self, cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True):
        print "executing: %s" % cmd
        subprocess.call(cmd, stdout=stdout, stderr=stderr, shell=shell)

    def run(self):
        old_data = 0
        while not rospy.is_shutdown():
            f = open("%s/value" % self.filepath)
            data = f.read()
            f.close()
            data = int(data.split("\n")[0])
#            if data != old_data:
#                print "inside!"
#                jsondata = {}
#                jsondata["value"] = data
#                print "jsondata: %s" % jsondata
#                dumped = json.dumps(jsondata)
#                self.aiPub.publish("HEY!!!!")
#                old_data = data
#                print "got past"
            tosend = {}
            tosend["value"] = data
            self.aiPub.publish(json.dumps(tosend))
            self.rate.sleep()

        rospy.spin()

if __name__ == "__main__":
    ir = IrReader()
    ir.run()

