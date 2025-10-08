数据集存放位置：/mmdet3d_kitti/data/kitti/  ...bag

环境配置信息
conda环境名称: ME5400
Python版本: 3.8.20
关键包版本:
PyTorch: 1.12.0+cu116 (CUDA 11.6版本)
MMCV: 2.0.1
MMDetection3D: 1.4.0
CUDA: 11.6

1. 创建conda环境
   conda create -n ME5400 python=3.8
   conda activate ME5400
2. 安装PyTorch (CUDA 11.6版本)
   pip install torch==1.12.0+cu116 torchvision==0.13.0+cu116 torchaudio==0.12.0+cu116 --extra-index-url https://download.pytorch.org/whl/cu116
3. 安装MMDetection3D相关包
   pip install mmcv==2.0.1
   pip install mmdet==3.0.0
   pip install mmdet3d==1.4.0
4. 安装其他必要依赖
   pip install open3d==0.16.0
   pip install opencv-python==4.12.0.88
   pip install numpy==1.24.4
   pip install matplotlib==3.5.3
   pip install scipy==1.10.1
   pip install scikit-learn==1.3.2
   pip install pandas==2.0.3
   pip install pillow==10.4.0
   pip install pyyaml==6.0.2
   pip install tqdm
   pip install terminaltables==3.1.10
   pip install shapely==1.8.5.post1
   pip install pyquaternion==0.9.9
   pip install trimesh==4.8.2
   pip install plyfile==1.0.3
   pip install imageio==2.35.1
   pip install fire==0.7.1
   pip install tensorboard==2.14.0
   pip install protobuf==5.29.5
5. 安装ROS相关包
   pip install rospkg==1.6.0
   pip install catkin-pkg==1.1.0
6. 验证安装
   python -c "import torch; print('PyTorch版本:', torch.__version__); print('CUDA可用:', torch.cuda.is_available())"
   python -c "import mmcv; print('MMCV版本:', mmcv.__version__)"
   python -c "import mmdet3d; print('MMDetection3D版本:', mmdet3d.__version__)"

运行

1. 启动roscore（如果还没启动）
   roscore &
2. 激活conda环境并运行ROS节点
   conda activate ME5400
   python kitti_pointpillars_bag_node.py
3. 在另一个终端播放bag文件
   rosbag play data/kitti/seq_0019_with_det.bag --clock --rate=0.5

发布消息/detection/kitti_tracking 为官方kitti tracking格式
