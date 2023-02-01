#!/usr/bin/env python3



import rospy
import math
from robp_msgs.msg import DutyCycles,Encoders
from geometry_msgs.msg import Twist

#task 2

#1. make executable chmod +x task1.py
#2. add node in launch file for task 1

Des_vx = None
Des_wz = None
DeltaEncLeft = None
DeltaEncRight = None
int_error1 = 0
int_error2 = 0
alpha1 = 0.02
alpha2 = 0.02
beta1 = 2*1e-6
beta2 = 2*1e-6

def callbackDesTw(msg):
    global Des_vx, Des_wz
    rospy.loginfo("In DesTw callback")
    Des_vx = msg.linear.x
    Des_wz = msg.angular.z

def callbackEnc(msg):
    global DeltaEncLeft,DeltaEncRight
    rospy.loginfo("In Encoder callback")
    DeltaEncLeft = msg.delta_encoder_left
    DeltaEncRight = msg.delta_encoder_right

def controller():
    global Des_vx, Des_wz, DeltaEncLeft, DeltaEncRight,int_error1,int_error2,alpha1,alpha2,beta1,beta2
    rospy.init_node('cartesian_controller_node', anonymous=True)
    rospy.Subscriber("/motor_controller/twist", Twist, callbackDesTw)

    rospy.Subscriber("/motor/encoders", Encoders, callbackEnc)

    pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)

    f = 10 # 10hz
    dt = 1/f
    rate = rospy.Rate(f) 
    

    rospy.sleep(8)
    
    while not rospy.is_shutdown():
        rospy.loginfo("controller of task 2 active")
        r = 0.04921 #m
        tpr = 3072
        b = 0.3 #m

        vw1 = 2*math.pi*r*f*DeltaEncLeft/tpr
        vw2 = 2*math.pi*r*f*DeltaEncRight/tpr

        # Est_v = (vw1+vw2)/2
        # Est_w = (vw2-vw1)/(2*b)

        Est_w1 = vw1/r
        Est_w2 = vw2/r

        Des_w2 = (b*Des_wz + Des_vx)/r
        Des_w1 = (Des_vx-b*Des_wz)/r

        error1 = Des_w1-Est_w1
        error2 = Des_w2-Est_w2
        
        int_error1 = int_error1 + error1 * dt
        

        int_error2 = int_error2 + error2 * dt
        

        rospy.loginfo(int_error1)
        rospy.loginfo(int_error2)


        introof = 4
        if abs(int_error1) > introof:
            int_error1 = 0#introof*int_error1/abs(int_error1)

        if abs(int_error2) > introof:
            int_error2 = 0#introof*int_error2/abs(int_error2)


        pwm1 = alpha1 * error1 + beta1 * int_error1
        pwm2 = alpha2 * error2 + beta2 * int_error2

        cmd = DutyCycles()
        cmd.duty_cycle_left = pwm1
        cmd.duty_cycle_right = pwm2
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

