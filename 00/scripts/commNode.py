#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import networking.client as client
import json


class CommNode:
    def __init__(self, ref, teammate, maincomputer):
        self.ref = ref
        self.teammate = teammate
        self.maincomputer = maincomputer
        rospy.init_node('commNode', anonymous=False)
        self.aiRefPub = rospy.Publisher('commAiRefChatter', String)
        self.aiTeamPub = rospy.Publisher('commAiTeamChatter', String)
        rospy.Subscriber("aiCommTeamChatter", String, self.fromAiToTeamCallback)
        rospy.Subscriber("aiCommDebugChatter", String, self.fromAiToDebugCallback)
        rospy.Subscriber("serverCommChatter", String, self.serverCallback)
        self.r = rospy.Rate(10)
        rospy.loginfo("started with ref: %s team: %s maincomptuer: %s" % (ref, teammate, maincomputer))
        #self.teamConnection = client.Client(self.teammate, 1337)
        self.debugConnection = client.Client(self.maincomputer, 1337)


    def fromAiToTeamCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "from ai to team: %s" % data.data)
        self.teamConnection.sendInfo(json.dumps(data.data))

    def fromAiToDebugCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "from ai to debug: %s" % data.data)
        self.debugConnection.sendInfo(json.dumps(data.data))


    def serverCallback(self, data):
        rospy.loginfo("serverCallback")
        received = json.loads(data.data)
        if str(received["address"]) == self.ref:
            rospy.loginfo("got from ref")
            tosend = {}
            tosend["ref"] = str(received["data"])
            self.aiRefPub.publish(json.dumps(tosend))
        elif str(received["address"]) == self.teammate:
            rospy.loginfo("got from team")
            self.aiTeamPub.publish(str(received["data"]))

        rospy.loginfo(rospy.get_caller_id() + "got: %s", data.data)

    def listen(self):
        rospy.spin()

    
if __name__ == "__main__":
    cn = CommNode(ref="192.168.1.117", teammate="192.168.1.117", maincomputer="192.168.1.117")
    cn.listen()
