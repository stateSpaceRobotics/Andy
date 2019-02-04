#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def ControllerCallback(controllerState):
    global notPaused
    global velocityPublisher

    if controllerState.buttons[buttons["START"]]:
        notPaused = notPaused ^ 1

    msg = Twist()

    x = -1.0 * (1.0 - controllerState.axes[axes["LT"]]) / 8.0

    if controllerState.axes[axes["RT"]] != 1.0:
        x = (1.0 - controllerState.axes[axes["RT"]]) / 8.0

    x *= notPaused
    z = controllerState.axes[axes["LS_LEFT_RIGHT"]] * notPaused

    msg.linear.x = x
    msg.angular.z = z * math.pi / 2.0

    velocityPublisher.publish(msg)


#Dictionaries for Controller State array accessing
buttons =  { "A" : 0, "B" : 1, "X" : 2, "Y" : 3, "LB" : 4, "RB" : 5, "BACK" : 6, "START" : 7, "POWER" : 8, "L3" : 9, "R3" : 10 }
axes =     { "LS_LEFT_RIGHT" : 0, "LS_UP_DOWN" : 1, "LT" : 2, "RS_LEFT_RIGHT" : 3, "RS_UP_DOWN" : 4, "RT" : 5, "DPAD_LEFT_RIGHT" : 6, "DPAD_UP_DOWN" : 7 }

velocityPublisher = 0
notPaused = 0

def main():
    global velocityPublisher
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/joy1", Joy, ControllerCallback)
    velocityPublisher = rospy.Publisher('/andy/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()