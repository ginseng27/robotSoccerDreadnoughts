#!/usr/bin/env python
from std_msgs.msg import String
import rospy
import json
import mtrctrl.wheelControls as wheelControls
import kickctrl.shooter as shooter

class ControlSystem:
    def __init__(self):
        self.WheelCtrls = wheelControls.WheelControls()
        self.kicker = shooter.Shooter()
        rospy.init_node('ControlSystem', anonymous=False)
        rospy.Subscriber("aiControlWheelsChatter", String, self.aiWheelsCallback)
        rospy.Subscriber("aiControlKickerChatter", String, self.aiKickCallback)
        rospy.Subscriber("aiControlDribblerChatter", String, self.aiDribblerCallback)
        self.kalmanPub = rospy.Publisher("controlKalmanChatter", String)
        self.aiPub = rospy.Publisher("controlAiChatter", String)
        self.rate = rospy.Rate(2)

    def aiDribblerCallback(self, data):
        formatted = json.loads(data.data)
        on = formatted["on"]
        self.WheelCtrls.dribbleOn(on)

    def aiWheelsCallback(self, data):
        formatted = json.loads(data.data)
        velocities = formatted["velocities"]
        self.WheelCtrls.moveGeneric(velocities)

    def aiKickCallback(self, data):
        formatted = json.loads(data.data)
        if formatted["kick"]:
            self.kicker.kickNow()

    def run(self):
        while not rospy.is_shutdown():
            data = self.WheelCtrls.actualRobotSpeed()
            tosend = {}
            tosend["velocities"] = data.tolist()
            self.kalmanPub.publish(json.dumps(tosend))
            self.rate.sleep()
        rospy.spin()




if __name__ == "__main__":
    cs = ControlSystem()
    cs.run()


