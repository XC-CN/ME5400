_base_ = [
    '../_base_/datasets/kitti-3d-3class.py',
    '../_base_/models/centerpoint_voxel01_second_secfpn_nus.py',
    '../_base_/schedules/cyclic-20e.py', '../_base_/default_runtime.py'
]

# 使用 nuScenes 预训练模型，但适配 KITTI 数据格式
point_cloud_range = [0, -40, -3, 70.4, 40, 1]
class_names = ['Pedestrian', 'Cyclist', 'Car']

# 修改模型配置以适配 KITTI
model = dict(
    data_preprocessor=dict(
        voxel_layer=dict(point_cloud_range=point_cloud_range)),
    pts_voxel_encoder=dict(type='HardSimpleVFE', num_features=4),  # KITTI 只有 4 维
    pts_bbox_head=dict(
        # 使用 nuScenes 的 10 个类别配置，但只关注 KITTI 相关的类别
        tasks=[
            dict(num_class=1, class_names=['car']),
            dict(num_class=1, class_names=['truck']),
            dict(num_class=1, class_names=['construction_vehicle']),
            dict(num_class=1, class_names=['bus']),
            dict(num_class=1, class_names=['trailer']),
            dict(num_class=1, class_names=['barrier']),
            dict(num_class=1, class_names=['motorcycle']),
            dict(num_class=1, class_names=['bicycle']),
            dict(num_class=1, class_names=['pedestrian']),
            dict(num_class=1, class_names=['traffic_cone']),
        ],
        bbox_coder=dict(
            type='CenterPointBBoxCoder',
            pc_range=point_cloud_range[:2],
            post_center_range=[0, -40, -10.0, 70.4, 40, 10.0],
            max_num=500,
            score_threshold=0.1,
            out_size_factor=8,
            voxel_size=[0.1, 0.1],
            code_size=7),  # KITTI 不需要速度信息
        # 修改损失函数权重
        loss_cls=dict(type='mmdet.GaussianFocalLoss', reduction='mean'),
        loss_bbox=dict(type='mmdet.L1Loss', reduction='mean', loss_weight=0.25),
    ),
    train_cfg=dict(
        pts=dict(
            grid_size=[1408, 1600, 40],
            voxel_size=[0.1, 0.1, 0.2],
            out_size_factor=8,
            dense_reg=1,
            gaussian_overlap=0.1,
            max_objs=500,
            min_radius=2,
            code_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])),  # 移除速度权重
    test_cfg=dict(
        pts=dict(
            post_center_limit_range=[0, -40, -10.0, 70.4, 40, 10.0],
            max_per_img=500,
            max_pool_nms=False,
            min_radius=[4, 12, 10],
            score_threshold=0.1,
            out_size_factor=8,
            voxel_size=[0.1, 0.1],
            nms_type='rotate',
            pre_max_size=1000,
            post_max_size=83,
            nms_thr=0.2)))

# 数据集配置
dataset_type = 'KittiDataset'
data_root = 'data/kitti/'
backend_args = None

# 简化的数据管道（用于推理）
test_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        backend_args=backend_args),
    dict(
        type='MultiScaleFlipAug3D',
        img_scale=(1333, 800),
        pts_scale_ratio=1,
        flip=False,
        transforms=[
            dict(
                type='GlobalRotScaleTrans',
                rot_range=[0, 0],
                scale_ratio_range=[1., 1.],
                translation_std=[0, 0, 0]),
            dict(type='RandomFlip3D'),
            dict(
                type='PointsRangeFilter', 
                point_cloud_range=point_cloud_range)
        ]),
    dict(type='Pack3DDetInputs', keys=['points'])
]

# 测试数据加载器
test_dataloader = dict(
    batch_size=1,
    num_workers=1,
    persistent_workers=True,
    drop_last=False,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        data_prefix=dict(pts='training/velodyne_reduced'),
        ann_file='kitti_infos_val.pkl',
        pipeline=test_pipeline,
        modality=dict(use_lidar=True, use_camera=False),
        test_mode=True,
        metainfo=dict(classes=class_names),
        box_type_3d='LiDAR',
        backend_args=backend_args))

# 评估器
val_evaluator = dict(
    type='KittiMetric',
    ann_file=data_root + 'kitti_infos_val.pkl',
    metric='bbox',
    backend_args=backend_args)
test_evaluator = val_evaluator
