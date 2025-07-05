#!/usr/bin/env python3
import rospy
from gimbal_bridge_node.msg import GimbalState

def gimbal_state_callback(msg):
    rospy.loginfo("Received: yaw=%.1f, pitch=%.1f, roll=%.1f, json=%s", 
                 msg.yaw, msg.pitch, msg.roll, msg.json_string)

def subscribe_gimbal_state():
    rospy.init_node('gimbal_state_subscriber')
    rospy.Subscriber('/gimbal/state', GimbalState, gimbal_state_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_gimbal_state()

