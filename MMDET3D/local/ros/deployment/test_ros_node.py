#!/usr/bin/env python3
"""
测试KITTI PointPillars ROS节点
"""

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
import tf.transformations as tf_trans


class TestROSNode:
    """测试ROS节点"""
    
    def __init__(self):
        rospy.init_node('test_kitti_detection', anonymous=True)
        
        # 创建发布器
        self.marker_pub = rospy.Publisher('/detection/bboxes_3d', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/detection/status', String, queue_size=10)
        
        # 统一线框颜色
        self.wireframe_color = (0.0, 0.447, 0.741, 1.0)
        self.class_colors = {
            'Car': self.wireframe_color,
            'Pedestrian': self.wireframe_color,
            'Cyclist': self.wireframe_color,
        }
        
        rospy.loginfo("测试ROS节点初始化完成")
    
    def create_test_markerarray(self) -> MarkerArray:
        """创建测试用的MarkerArray"""
        marker_array = MarkerArray()
        
        # 创建时间戳
        current_time = rospy.Time.now()
        
        # 创建几个测试检测结果
        test_detections = [
            {'class': 'Car', 'x': 10.0, 'y': 0.0, 'z': 0.0, 'length': 4.0, 'width': 2.0, 'height': 1.5, 'yaw': 0.0},
            {'class': 'Pedestrian', 'x': 5.0, 'y': 3.0, 'z': 0.0, 'length': 0.8, 'width': 0.6, 'height': 1.7, 'yaw': 0.0},
            {'class': 'Cyclist', 'x': 15.0, 'y': -2.0, 'z': 0.0, 'length': 2.0, 'width': 0.8, 'height': 1.5, 'yaw': 0.5},
        ]
        
        for i, detection in enumerate(test_detections):
            # 创建Marker
            marker = Marker()
            marker.header.frame_id = "lidar"
            marker.header.stamp = current_time
            marker.ns = "detection"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            
            # 设置位置
            marker.pose.position.x = detection['x']
            marker.pose.position.y = detection['y']
            marker.pose.position.z = detection['z']
            
            # 设置姿态 (旋转)
            quat = tf_trans.quaternion_from_euler(0, 0, detection['yaw'])
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            
            # 线宽
            marker.scale.x = 0.05
            marker.scale.y = 0.0
            marker.scale.z = 0.0
            
            # 设置颜色
            color = self.class_colors[detection['class']]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # 设置生命周期
            marker.lifetime = rospy.Duration(0)
            marker.frame_locked = False
            
            # 构造线框
            half_l = detection['length'] / 2.0
            half_w = detection['width'] / 2.0
            half_h = detection['height'] / 2.0
            corners = [
                (half_l, half_w, half_h),
                (half_l, -half_w, half_h),
                (-half_l, -half_w, half_h),
                (-half_l, half_w, half_h),
                (half_l, half_w, -half_h),
                (half_l, -half_w, -half_h),
                (-half_l, -half_w, -half_h),
                (-half_l, half_w, -half_h),
            ]
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            for start_idx, end_idx in edges:
                start = corners[start_idx]
                end = corners[end_idx]
                marker.points.append(Point(*start))
                marker.points.append(Point(*end))

            marker_array.markers.append(marker)
        
        return marker_array
    
    def run_test(self):
        """运行测试"""
        rospy.loginfo("开始测试...")
        
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # 创建测试MarkerArray
            marker_array = self.create_test_markerarray()
            
            # 发布MarkerArray
            self.marker_pub.publish(marker_array)
            
            # 发布状态
            status_msg = String()
            status_msg.data = f"测试模式 - 发布了 {len(marker_array.markers)} 个检测结果"
            self.status_pub.publish(status_msg)
            
            rospy.loginfo(f"发布了 {len(marker_array.markers)} 个测试检测结果")
            
            rate.sleep()


def main():
    """主函数"""
    try:
        test_node = TestROSNode()
        test_node.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试节点被中断")
    except Exception as e:
        rospy.logerr(f"测试节点运行错误: {e}")


if __name__ == '__main__':
    main()
