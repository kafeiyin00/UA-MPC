#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os
from datetime import datetime

class EvaluateCompleteNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('EvaluateComplete', anonymous=True)
        
        # 日志文件路径
        log_dir = rospy.get_param("~log_file_path", "/tmp")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file_path = os.path.join(log_dir, f"EvaluateComplete_log_{date_str}.txt")

        # 创建发布者
        self.pub_pcl = rospy.Publisher('/cloud_registered_copy', PointCloud2, queue_size=10)

        # 创建订阅者
        self.sub_pcl = rospy.Subscriber('/cloud_registered', PointCloud2, self.callback_pcl)
        
        # 初始化参数
        self.point_clouds = []  # 存储点云数据
        self.voxel_size = 0.3  # 体素大小
        self.message_count = 0  # 接收到的消息计数
        self.message_count_max = 50  # 合并点云的消息数量
    
    def pointcloud_to_array(self, msg):
        """
        将 PointCloud2 消息转换为 NumPy 数组
        """
        point_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.array(list(point_generator))

    def merge_pointclouds(self):
        """
        合并存储的点云
        """
        if len(self.point_clouds) > 0:
            return np.vstack(self.point_clouds)  # 垂直堆叠所有点云
        else:
            return np.array([])

    def calculate_voxel_count(self, points):
        """
        计算点云占据的体素数量
        """
        if points.size == 0:
            return 0

        # 使用体素大小对点云进行量化
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        # 获取唯一的体素索引
        unique_voxels = np.unique(voxel_indices, axis=0)

        # 返回体素的数量
        return unique_voxels.shape[0]

    def callback_pcl(self, msg):
        """
        点云回调函数
        """
        # 转换并存储点云
        points = self.pointcloud_to_array(msg)
        self.point_clouds.append(points)
        self.message_count += 1

        # 转发原始点云
        self.pub_pcl.publish(msg)

        # 每 50 条消息合并并处理
        if self.message_count % self.message_count_max == 0:
            merged_cloud = self.merge_pointclouds()
            voxel_count = self.calculate_voxel_count(merged_cloud)
            rospy.loginfo(f"Merged point cloud size: {merged_cloud.shape[0]}")
            rospy.loginfo(f"Occupied voxels: {voxel_count}")

            # 写入日志文件
            with open(self.log_file_path, 'a') as log_file:
                log_file.write(f"{voxel_count}\n")

            # 清空存储的点云
            self.point_clouds = []

    def run(self):
        """
        主运行循环
        """
        rospy.spin()  # 使用 spin 保持订阅者运行

if __name__ == '__main__':
    try:
        node = EvaluateCompleteNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")