#!/usr/bin/env python3

import rospy
import math
from adafruit_servokit import ServoKit
kit = ServoKit(channels = 16)
kit.servo[0].actuation_range = 180
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState 

def callback(data):
    
    #print(data.position[0])
    #rospy.loginfo(rospy.get_caller_id() + 'This is the fist position', data.position[0])
    joint_0 = (math.degrees(data.position[0]) + 90)
    print(joint_0)
    kit.servo[0].angle = joint_0
    time.sleep(0.5)
    joint_1 = (math.degrees(data.position[1]) + 90)
    joint_2 = (math.degrees(data.position[2]) + 90)
    joint_3 = (math.degrees(data.position[3]) + 90)
    joint_4 = (math.degrees(data.position[4]) + 90)
    kit.servo[1].angle = joint_1
    kit.servo[2].angle = joint_2
    kit.servo[3].angle = joint_3
    kit.servo[4].angle = joint_4
    
    
    
    
    print( joint_0 , joint_1 ,joint_2, joint_3, joint_4)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hardware_interface', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
