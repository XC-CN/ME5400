#!/usr/bin/env python3

import rospy
import numpy as np
import os
import sys
from pathlib import Path
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud

class NuScenesRvizVisualizer:
    def __init__(self, dataroot, version='v1.0-trainval'):
        """初始化nuScenes Rviz可视化器"""
        
        # 初始化ROS节点
        rospy.init_node('nuscenes_rviz_visualizer', anonymous=True)
        
        # 初始化nuScenes数据集
        print("正在加载nuScenes数据集...")
        self.nusc = NuScenes(version=version, dataroot=dataroot, verbose=False)
        print(f"数据集加载完成！包含 {len(self.nusc.scene)} 个场景")
        
        # 初始化ROS发布器
        self.lidar_pub = rospy.Publisher('/nuscenes/lidar', PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher('/nuscenes/pose', PoseStamped, queue_size=1)
        
        # 初始化TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 当前场景和样本索引
        self.current_scene_idx = 0
        self.current_sample_idx = 0
        
        print("nuScenes LiDAR可视化器初始化完成！")
        print("开始连续播放LiDAR数据...")
    
    def load_sample_data(self, scene_idx, sample_idx):
        """加载指定场景和样本的数据"""
        
        if scene_idx >= len(self.nusc.scene):
            print(f"场景索引超出范围: {scene_idx}")
            return False
        
        scene = self.nusc.scene[scene_idx]
        
        # 获取场景中的样本
        samples = []
        current_sample = self.nusc.get('sample', scene['first_sample_token'])
        while current_sample:
            samples.append(current_sample)
            if current_sample['next']:
                current_sample = self.nusc.get('sample', current_sample['next'])
            else:
                break
        
        if sample_idx >= len(samples):
            print(f"样本索引超出范围: {sample_idx}")
            return False
        
        sample = samples[sample_idx]
        
        # 发布激光雷达数据
        self.publish_lidar_data(sample)
        
        # 发布位姿数据
        self.publish_pose_data(sample)
        
        print(f"场景 {scene_idx+1}/{len(self.nusc.scene)} - 样本 {sample_idx+1}/{len(samples)}")
        print(f"场景: {scene['name']} - {scene['description']}")
        
        return True
    
    def publish_lidar_data(self, sample):
        """发布激光雷达数据"""
        try:
            # 获取激光雷达数据
            lidar_data = self.nusc.get('sample_data', sample['data']['LIDAR_TOP'])
            
            # 加载点云数据
            pc_path = os.path.join(self.nusc.dataroot, lidar_data['filename'])
            pc = LidarPointCloud.from_file(pc_path)
            
            # 转换为ROS PointCloud2消息
            points = pc.points[:3].T  # 取x, y, z坐标
            
            # 创建PointCloud2消息
            cloud_msg = PointCloud2()
            cloud_msg.header = Header()
            cloud_msg.header.stamp = rospy.Time.from_sec(lidar_data['timestamp'] / 1e6)
            cloud_msg.header.frame_id = 'lidar'
            
            # 设置点云数据
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            cloud_msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.data = points.astype(np.float32).tobytes()
            cloud_msg.is_dense = True
            
            # 发布点云数据
            self.lidar_pub.publish(cloud_msg)
            
        except Exception as e:
            print(f"发布激光雷达数据失败: {e}")
    
    def publish_pose_data(self, sample):
        """发布位姿数据"""
        try:
            # 获取LiDAR数据以获取时间戳
            lidar_data = self.nusc.get('sample_data', sample['data']['LIDAR_TOP'])
            
            # 创建简单的位姿消息（车辆位置设为原点）
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.from_sec(lidar_data['timestamp'] / 1e6)
            pose_msg.header.frame_id = 'map'
            
            # 设置位置为原点
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            
            # 设置姿态为单位四元数
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            # 发布位姿
            self.pose_pub.publish(pose_msg)
            
            # 发布TF变换
            self.publish_tf_transforms(lidar_data['timestamp'])
            
        except Exception as e:
            print(f"发布位姿数据失败: {e}")
    
    def publish_tf_transforms(self, timestamp):
        """发布TF变换"""
        try:
            # 发布map到lidar的变换
            t = TransformStamped()
            t.header.stamp = rospy.Time.from_sec(timestamp / 1e6)
            t.header.frame_id = 'map'
            t.child_frame_id = 'lidar'
            
            # 设置变换为单位变换
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            print(f"发布TF变换失败: {e}")
    
    def run(self):
        """运行连续LiDAR可视化器"""
        print("开始连续播放LiDAR数据...")
        print("按Ctrl+C停止播放")
        print("")
        
        # 主循环 - 连续播放所有样本
        rate = rospy.Rate(2)  # 2Hz，每0.5秒播放一帧
        
        while not rospy.is_shutdown():
            try:
                # 加载当前样本数据
                success = self.load_sample_data(self.current_scene_idx, self.current_sample_idx)
                
                if not success:
                    print("所有数据播放完毕！")
                    break
                
                # 移动到下一个样本
                self.current_sample_idx += 1
                
                # 如果当前场景的样本播放完毕，移动到下一个场景
                if self.current_scene_idx < len(self.nusc.scene):
                    scene = self.nusc.scene[self.current_scene_idx]
                    samples = self.nusc.get('sample', scene['first_sample_token'])
                    sample_list = []
                    while samples:
                        sample_list.append(samples)
                        if samples['next']:
                            samples = self.nusc.get('sample', samples['next'])
                        else:
                            break
                    
                    if self.current_sample_idx >= len(sample_list):
                        self.current_scene_idx += 1
                        self.current_sample_idx = 0
                        if self.current_scene_idx >= len(self.nusc.scene):
                            print("所有场景播放完毕！")
                            break
                
                rate.sleep()
                
            except KeyboardInterrupt:
                print("\n用户中断播放")
                break
            except Exception as e:
                print(f"播放过程中出现错误: {e}")
                break
        
        print("LiDAR可视化器已停止")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python3 nuscenes_rviz_visualizer.py <dataroot>")
        sys.exit(1)
    
    dataroot = sys.argv[1]
    
    if not os.path.exists(dataroot):
        print(f"数据路径不存在: {dataroot}")
        sys.exit(1)
    
    try:
        visualizer = NuScenesRvizVisualizer(dataroot)
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
