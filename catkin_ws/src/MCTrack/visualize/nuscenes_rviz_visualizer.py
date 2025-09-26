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
from nuscenes.utils.data_classes import LidarPointCloud, Box
from visualization_msgs.msg import Marker, MarkerArray
from pyquaternion import Quaternion

# ===== 可配置参数（便于修改） =====
# 第一个播放场景的索引（0基）
START_SCENE_INDEX = 4  # 默认从第4个场景开始
# 播放帧率（Hz）
PLAYBACK_HZ = 5      # 默认10Hz

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
        self.det_pub = rospy.Publisher('/nuscenes/detections', MarkerArray, queue_size=1)
        
        # 初始化TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 当前场景和样本索引（由顶部常量控制）
        self.current_scene_idx = START_SCENE_INDEX
        self.current_sample_idx = 0
        
        print("nuScenes LiDAR可视化器初始化完成！")
        print("开始连续播放LiDAR数据...")
        
        # 预加载检测结果（优先使用centerpoint/val.json，其次largekernel/test.json）
        self.det_results = {}
        try:
            self._load_detection_results()
        except Exception as e:
            print(f"检测结果加载失败: {e}")
    
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
        
        try:
            # 场景切换时输出连贯性与地点信息
            if sample_idx == 0:
                try:
                    log_curr = self.nusc.get('log', scene['log_token'])
                    location = log_curr.get('location', '未知')
                except Exception:
                    location = '未知'
                if scene_idx > 0:
                    prev_scene = self.nusc.scene[scene_idx - 1]
                    contiguous = (prev_scene.get('log_token') == scene.get('log_token'))
                    print(f"场景切换 -> {scene['name']} | 地点: {location} | 连贯性: {'连贯(同一log)' if contiguous else '不连贯(不同log)'}")
                else:
                    print(f"开始播放 -> {scene['name']} | 地点: {location}")
            
            # 发布激光雷达数据
            print("--------------------------------")
            print(f"正在加载样本 {sample_idx+1} 的LiDAR数据...")
            self.publish_lidar_data(sample)
            print(f"LiDAR数据加载完成")
            
            # 发布位姿数据
            self.publish_pose_data(sample)
            
            # 发布检测框（如有）
            self.publish_detection_markers(scene, sample)
            
            print(f"场景 {scene_idx+1}/{len(self.nusc.scene)} - 样本 {sample_idx+1}/{len(samples)}")
            print(f"场景: {scene['name']} - {scene['description']}")
            
        except Exception as e:
            print(f"加载样本数据时出错: {e}")
            return False
        
        return True
    
    def publish_lidar_data(self, sample):
        """发布激光雷达数据"""
        try:
            # 获取激光雷达数据
            lidar_data = self.nusc.get('sample_data', sample['data']['LIDAR_TOP'])
            
            # 加载点云数据（不输出完整路径/文件名）
            pc_path = os.path.join(self.nusc.dataroot, lidar_data['filename'])
            
            pc = LidarPointCloud.from_file(pc_path)
            print(f"  点云加载成功，点数: {pc.points.shape[1]}")
            
            # 转换为ROS PointCloud2消息
            points = pc.points[:3].T  # 取x, y, z坐标
            
            # 创建PointCloud2消息
            cloud_msg = PointCloud2()
            cloud_msg.header = Header()
            # 使用当前ROS时间，避免TF旧数据问题
            cloud_msg.header.stamp = rospy.Time.now()
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
            # 使用当前ROS时间
            pose_msg.header.stamp = rospy.Time.now()
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
            # 使用当前ROS时间
            t.header.stamp = rospy.Time.now()
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
    
    def _load_detection_results(self):
        """加载检测结果JSON，支持centerpoint/val.json或largekernel/test.json"""
        base_dir = Path('data/base_version/nuscenes')
        candidates = [
            base_dir / 'centerpoint' / 'val.json',
            base_dir / 'largekernel' / 'test.json',
        ]
        det_path = None
        for p in candidates:
            if p.exists():
                det_path = p
                break
        if det_path is None:
            print("未发现可用的检测结果JSON，跳过检测框叠加")
            return
        import json
        with open(det_path, 'r') as f:
            data = json.load(f)
        # 兼容两种常见结构：{results:{token:[...]}} 或 直接 {token:[...]}
        if 'results' in data:
            self.det_results = data['results']
        else:
            self.det_results = data
        print("检测结果已加载")

    def publish_detection_markers(self, scene, sample):
        """根据检测结果发布MarkerArray到/nuscenes/detections，与点云对齐（LIDAR_TOP坐标系）"""
        if not self.det_results:
            return
        sample_token = sample.get('token')
        dets = self.det_results.get(sample_token, [])
        if not dets:
            # 当前样本无检测，发布空数组以清空旧标记
            self.det_pub.publish(MarkerArray())
            return
        # 获取当前样本的LIDAR标定与自车位姿，用于全局->传感器坐标变换
        sd = self.nusc.get('sample_data', sample['data']['LIDAR_TOP'])
        ego_pose = self.nusc.get('ego_pose', sd['ego_pose_token'])
        calib = self.nusc.get('calibrated_sensor', sd['calibrated_sensor_token'])
        q_ego = Quaternion(ego_pose['rotation'])
        q_calib = Quaternion(calib['rotation'])
        t_ego = np.array(ego_pose['translation'])
        t_calib = np.array(calib['translation'])
        
        ma = MarkerArray()
        now = rospy.Time.now()
        for i, d in enumerate(dets):
            try:
                # 构建全局坐标系下的Box
                center = np.array(d.get('translation', [0,0,0]))
                size = np.array(d.get('size', [0,0,0]))  # [w, l, h]
                rot = d.get('rotation', [1,0,0,0])      # [w,x,y,z]
                name = d.get('detection_name', 'obj')
                score = float(d.get('detection_score', d.get('score', 0.5)))
                box = Box(center=center, size=size, orientation=Quaternion(rot), name=name, score=score)
                
                # 全局->自车
                box.translate(-t_ego)
                box.rotate(q_ego.inverse)
                # 自车->传感器(LIDAR_TOP)
                box.translate(-t_calib)
                box.rotate(q_calib.inverse)
                
                # 构造Marker（CUBE）
                m = Marker()
                m.header.frame_id = 'lidar'
                m.header.stamp = now
                m.ns = 'nuscenes_dets'
                m.id = i
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = float(box.center[0])
                m.pose.position.y = float(box.center[1])
                m.pose.position.z = float(box.center[2])
                q = box.orientation
                m.pose.orientation.w = float(q.w)
                m.pose.orientation.x = float(q.x)
                m.pose.orientation.y = float(q.y)
                m.pose.orientation.z = float(q.z)
                # Box.size: [w, l, h] -> RViz: x=length, y=width, z=height
                m.scale.x = float(box.size[1])
                m.scale.y = float(box.size[0])
                m.scale.z = float(box.size[2])
                # 颜色：红色，透明度随分数变化
                alpha = max(0.2, min(1.0, 0.3 + 0.7*score))
                m.color.r = 1.0
                m.color.g = 0.1
                m.color.b = 0.1
                m.color.a = alpha
                # 寿命：一帧
                m.lifetime = rospy.Duration(0)
                ma.markers.append(m)
            except Exception:
                continue
        self.det_pub.publish(ma)

    def run(self):
        """运行连续LiDAR可视化器"""
        print("开始连续播放LiDAR数据...")
        print("按Ctrl+C停止播放")
        print("")
        
        # 主循环 - 连续播放所有样本
        rate = rospy.Rate(PLAYBACK_HZ)  # 由顶部常量控制帧率
        
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
