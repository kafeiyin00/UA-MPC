#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
import os
from datetime import datetime

#/quad_0/lidar_slam/odom GT
#/aft_mapped_to_init

class EvaluateATENode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('EvaluateATE', anonymous=True)
        
        self.log_file_path = rospy.get_param("~log_file_path", "/tmp")
        
        # 获取日志文件路径
        log_dir = rospy.get_param("~log_file_path", "/tmp")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file_path = os.path.join(log_dir, f"EvaluateATE_log_{timestamp}.txt")
        
        # 打开日志文件
        self.log_file = open(self.log_file_path, "w")
        self.log_file.write("Timestamp, Odom1_X, Odom1_Y, Odom1_Z, Odom2_X, Odom2_Y, Odom2_Z\n")

        rospy.loginfo(f"Logging synchronized odometry data to: {self.log_file_path}")

        # 订阅两个 Odometry 消息
        self.sub_gt = Subscriber("/quad_0/lidar_slam/odom", Odometry)
        self.sub_slam = Subscriber("/aft_mapped_to_init", Odometry)
        
        # 设置同步器
        self.ts = ApproximateTimeSynchronizer([self.sub_gt, self.sub_slam], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.odom_callback)

        
    def odom_callback(self, odom_gt, odom_slam):
        """
        回调函数，同步后的 odometry 消息处理
        """
        # 提取第一个 odometry 消息的位置信息
        pos1 = odom_gt.pose.pose.position
        x1, y1, z1 = pos1.x, pos1.y, pos1.z

        # 提取第二个 odometry 消息的位置信息
        pos2 = odom_slam.pose.pose.position
        x2, y2, z2 = pos2.x, pos2.y, pos2.z

        # 获取当前时间戳
        timestamp = rospy.Time.now().to_sec()

        # 写入日志文件
        self.log_file.write(f"{timestamp}, {x1}, {y1}, {z1}, {x2}, {y2}, {z2}\n")
        # rospy.loginfo(f"Synchronized: Odom1({x1}, {y1}, {z1}) | Odom2({x2}, {y2}, {z2})")
        
    def run(self):
        """
        主运行循环
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EvaluateATENode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")