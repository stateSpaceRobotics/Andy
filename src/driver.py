#!/usr/bin/env python
import socket
import rospy
import math
from geometry_msgs.msg import Twist

#Wrapper class for the UDP Socket
class UDP_Handler:

    def __init__(self, ip, port):
        self.__ip = ip
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sendMessage(self, message):
        self.__sock.sendto(message, (self.__ip, self.__port))


#Determine the sign (positive or negative) of the argument
def sign(x):
    if x < 0.0:
        return -1.0
    return 1.0


#Build the message of motor commands to send over UDP
def UDPSendMsg(twist_msg):
    x = twist_msg.linear.x
    z = twist_msg.angular.z

    #If there's no angular component (z == 0) the motors just receive the linear component
    Vright = x
    Vleft = x

    wheelOffset = 0.115
    wheelRadius = 0.06

    #If there is an angular component we have to calculate the velocity for the left and right side motors to achieve the desired arc
    if z != 0:
        r = abs(x/z)
        zabs = abs(z)
        signx = sign(x)
        signz = sign(z)

        #Calculate the radii for the arcs followed by the right side and the left side
        rright = r + signx * signz * wheelOffset
        rleft = r + signx * signz * -1.0 * wheelOffset        

        #Calculate the linear velocity necessary to achieve the corresponding arcs
        Vright = signx * zabs * rright
        Vleft = signx * zabs * rleft
 
    #Convert the calculated linear velocities into angular velocities because that is what the embedded side wants
    Vright = -1.0 * Vright / wheelRadius    #Note that the right side is multiplied by -1. This is because direction is flipped due to the motor's physical orientation
    Vleft = Vleft / wheelRadius
    
    #Construct and send the message to the embedded side via UDP
    message = str(FRONT_LEFT_DRIVE) + "," + str(Vleft) + "\n"
    message += str(BACK_LEFT_DRIVE) + "," + str(Vleft) + "\n"
    message += str(FRONT_RIGHT_DRIVE) + "," + str(Vright) + "\n"
    message += str(BACK_RIGHT_DRIVE) + "," + str(Vright) + "\n"
    rospy.loginfo(message)
    set_point.sendMessage(message)


#Motor Indices on embedded side
FRONT_LEFT_DRIVE = 0
FRONT_RIGHT_DRIVE = 1
BACK_RIGHT_DRIVE = 2
BACK_LEFT_DRIVE = 3

set_point = UDP_Handler("192.168.0.32", 3233)

def main():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/andy/cmd_vel", Twist, UDPSendMsg)

    rospy.spin()

if __name__ == '__main__':
    main()