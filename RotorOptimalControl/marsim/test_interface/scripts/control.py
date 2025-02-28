#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def float_publisher():
    # 初始化节点
    rospy.init_node('float_publisher', anonymous=True)

    # 创建一个Publisher，发布到'/float_topic'话题，消息类型为Float32
    pub = rospy.Publisher('rotorspeedcontrol', Float32, queue_size=10)

    # 设置循环的频率
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        float_msg = Float32()
        float_msg.data = 1  # 你可以修改这里的值或根据需要动态改变它

        # 发布消息
        rospy.loginfo("Publishing float data: %f", float_msg.data)
        pub.publish(float_msg)

        # 等待直到下一次迭代
        rate.sleep()

if __name__ == '__main__':
    try:
        float_publisher()
    except rospy.ROSInterruptException:
        pass