#!/usr/bin/env python3

import sys,time
import PIDFunctions as pid
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

Kp = 25
Ki = 0.8
Kd = 0.1


PIDControl = pid.PID_Controller(Kp, Ki, Kd)

class SelfBalance:
    def __init__(self):
        self.pubVel = rospy.Publisher("cmd_vel", Twist, queue_size =1)
        self.pubYaw = rospy.Publisher("yaw", Float32, queue_size =1)
        
        self.subIMU = rospy.Subscriber("imu", Imu, self.callback_IMU)
        self.subKp = rospy.Subscriber("Kp", Float32, self.callback_Kp)
        self.subKi = rospy.Subscriber("Ki", Float32, self.callback_Ki)
        self.subKd = rospy.Subscriber("Kd", Float32, self.callback_Kd)

        self.x_velMin = -0.01
        self.x_velMax = 0
        self.y_min = -0.01
        self.y_max = -0.001
        self.prevY = 0
        self.delY = 0
        self.Kp = 25
        self.Ki = 0.8
        self.Kd = 0.1
        self.PIDControl = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

    def callback_IMU(self, data):
        setPoint = 0
        y = data.orientation.y
        self.delY = y - self.prevY

        if self.delY > self.y_max:
            self.y_max = self.delY
        elif self.delY < self.prevY:
            self.y_min = self.delY
        
        vel = Twist()
        x_vel = -self.PIDControl.getCorrection(setPoint, y)
        
        if x_vel > self.x_velMax:
            self.x_velMax = x_vel
        elif x_vel < self.x_velMax:
            self.x_velMin = x_vel

        if x_vel > 26:
            x_vel = 26
        elif x_vel < -26:
            x_vel = -26
        
        vel.linear.x = x_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        self.pubVel.publish(vel)
        self.prevY = y
        self.pubYaw.publish(y)

    def callback_Kp(self, data):
        self.Kp = data.data
        self.PIDControl = pid.PID_Controller(self.Kp, self.Ki, self.Kd)
    def callback_Ki(self, data):
        self.Ki = data.data
        self.PIDControl = pid.PID_Controller(self.Kp, self.Ki, self.Kd)
    def callback_Kd(self, data):
        self.Kd = data.data
        self.PIDControl = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

def main(args):
    rospy.init_node('SelfBalance', anonymous=True)
    sb = SelfBalance()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS")

if __name__ == '__main__':
    main(sys.argv)