#!/usr/bin/env python3
import rospy
from gimbal_bridge_node.msg import GimbalCmd
import math
import time

def publish_gimbal_cmd():
    pub = rospy.Publisher('/gimbal/cmd', GimbalCmd, queue_size=10)
    rospy.init_node('gimbal_cmd_publisher')
    rate = rospy.Rate(20)  # 5Hz 发布频率

    # 初始化变量
    start_time = rospy.Time.now().to_sec()  # 获取当前时间（秒）
    yaw_frequency = 0.11  # yaw 的频率（Hz）
    pitch_frequency = 0.13  # pitch 的频率（Hz）
    yaw_amplitude = 134  # yaw 的振幅（度）
    pitch_amplitude = (24 - (-89)) / 2  # pitch 的振幅（度）
    yaw_offset = 0  # yaw 的偏移量
    pitch_offset = (-89 + 24) / 2  # pitch 的偏移量

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()  # 获取当前时间（秒）
        elapsed_time = current_time - start_time  # 经过的时间（秒）

        # 计算 yaw 和 pitch 的值
        yaw = yaw_amplitude * math.sin(2 * math.pi * yaw_frequency * elapsed_time) + yaw_offset
        pitch = pitch_amplitude * math.sin(2 * math.pi * pitch_frequency * elapsed_time) + pitch_offset

        # 创建并发布消息
        msg = GimbalCmd()
        if math.fmod(current_time-start_time, 10) < 5:
            msg.gimbal_state_machine = 0
        else:
            msg.gimbal_state_machine = 1
        msg.yaw = yaw
        msg.pitch = pitch
        msg.json_string = '{"status":"normal"}'
        pub.publish(msg)

        # 打印日志
        rospy.loginfo("Published: yaw=%.1f, pitch=%.1f, state = %d, json=%s",
                      msg.yaw, msg.pitch, msg.gimbal_state_machine, msg.json_string)

        # 等待下一个周期
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gimbal_cmd()
    except rospy.ROSInterruptException:
        pass