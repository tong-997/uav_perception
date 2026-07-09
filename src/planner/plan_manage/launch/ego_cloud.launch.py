#完全修改的launch文件
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --------- Arguments ----------
   
    drone_id         = LaunchConfiguration('drone_id', default='0')
    odom_topic       = LaunchConfiguration('odom_topic', default='/vins/odometry')
    camera_pose_topic= LaunchConfiguration('camera_pose_topic', default='/ego/camera_pose_ps')
    depth_topic      = LaunchConfiguration('depth_topic', default='/unused') # 点云模式下不用深度图
    cloud_topic      = LaunchConfiguration('cloud_topic', default='/ego/points')
    use_mockamap     = LaunchConfiguration('use_mockamap', default='false')  # true=起模拟地图/渲染
    flight_type      = LaunchConfiguration('flight_type', default='1')
    #+
    poscmd_odom_topic = LaunchConfiguration('poscmd_odom_topic', default='/poscmd_odom')
    # map 尺寸（给 advanced_param / simulator 用）
    map_size_x = LaunchConfiguration('map_size_x', default='50.0')
    map_size_y = LaunchConfiguration('map_size_y', default='25.0')
    map_size_z = LaunchConfiguration('map_size_z', default='2.0')

    # 声明
    args = [
        DeclareLaunchArgument('drone_id', default_value=drone_id),
        DeclareLaunchArgument('odom_topic', default_value=odom_topic),
        DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('cloud_topic', default_value=cloud_topic),
        DeclareLaunchArgument('use_mockamap', default_value=use_mockamap),
        DeclareLaunchArgument('flight_type', default_value=flight_type),
        DeclareLaunchArgument('map_size_x', default_value=map_size_x),
        DeclareLaunchArgument('map_size_y', default_value=map_size_y),
        DeclareLaunchArgument('map_size_z', default_value=map_size_z),
        DeclareLaunchArgument('poscmd_odom_topic', default_value=poscmd_odom_topic)
    ]

    # --------- Include advanced_param (把真实/模拟输入显式传下去) ----------
    advanced_param = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')
        ),
        launch_arguments={
            'drone_id':        drone_id,
            'map_size_x_':     map_size_x,
            'map_size_y_':     map_size_y,
            'map_size_z_':     map_size_z,
            'odometry_topic':  odom_topic,
            'obj_num_set':     TextSubstitution(text='10'),
            'name_of_ego_planner_node': PythonExpression(["'ego_planner_' + str(", drone_id, ")"]),

            # 关键：把你想要的输入显式给下去（真实链路或模拟链路）
            'camera_pose_topic': camera_pose_topic,
            'depth_topic':       depth_topic,
            'cloud_topic':       cloud_topic,

            # 飞行 & 规划参数（按需改）
            'flight_type':      flight_type,
            'planning_horizon': TextSubstitution(text='7.5'),
            'max_vel':          TextSubstitution(text='2.0'),
            'max_acc':          TextSubstitution(text='6.0'),
            'use_distinctive_trajs': TextSubstitution(text='True'),

            # 相机内参（与你深度节点一致）
            'fx': TextSubstitution(text='391.50625'),
            'fy': TextSubstitution(text='395.18355'),
            'cx': TextSubstitution(text='335.18186'),
            'cy': TextSubstitution(text='217.7217'),
        }.items()
    )

    # --------- poscmd_2_odom（修正空 remap：command 直连到 /drone_<id>_planning/pos_cmd） ----------
    poscmd_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=PythonExpression(["'poscmd_2_odom_' + str(", drone_id, ")"]),
        output='screen',
        remappings=[
            # 直接给出实际话题，避免出现 "command:=" 空值
            ('command',  PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/pos_cmd'"])),
            #这个不要占用/vins/odometry
            ('odometry', poscmd_odom_topic),
        ]
    )

    # --------- 仅在 use_mockamap=true 时才起模拟地图和渲染 ----------
    map_generator = Node(
        package='map_generator', executable='random_forest', name='random_forest',
        output='screen',
        parameters=[{'map/x_size': 26.0},
                    {'map/y_size': 20.0},
                    {'map/z_size': 3.0},
                    {'map/resolution': 0.1},
                    {'ObstacleShape/seed': 1.0},
                    {'map/obs_num': 250},
                    {'ObstacleShape/lower_rad': 0.5},
                    {'ObstacleShape/upper_rad': 0.7},
                    {'ObstacleShape/lower_hei': 0.0},
                    {'ObstacleShape/upper_hei': 3.0},
                    {'map/circle_num': 250},
                    {'ObstacleShape/radius_l': 0.7},
                    {'ObstacleShape/radius_h': 0.5},
                    {'ObstacleShape/z_l': 0.7},
                    {'ObstacleShape/z_h': 0.8},
                    {'ObstacleShape/theta': 0.5},
                    {'pub_rate': 1.0},
                    {'min_distance': 0.8}],
        condition=IfCondition(use_mockamap)
    )

    pcl_render = Node(
        package='local_sensing_node', executable='pcl_render_node',
        name=PythonExpression(["'pcl_render_node_' + str(", drone_id, ")"]),
        output='screen',
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry',   odom_topic),
            # 输出到 pcl_render_node/depth /cloud 给模拟链路使用
            ('depth', '/ego/depth_image'),
            ('cloud', '/ego/points'),
        ],
        condition=IfCondition(use_mockamap)
    )
    
    #----------tf Node---------
    #base_link=body imu坐标系，world世界坐标系=map，统一用world
    tf_node = Node(
        package='ego_planner',
	executable='ego_tf_broadcaster.py',   # ament_cmake 脚本安装后用脚本名
	name='ego_tf_broadcaster',
	output='screen',
	parameters=[{
            'odom_topic': odom_topic,
	    'world_frame': 'world',
	    'publish_world_tf': True, #打开动态TF
	    'zero_on_first': True, #零帧对齐
	    'base_frame': 'base_link',
	    'camera_frame': 'default_cam',
	    'camera_optical_frame': 'default_cam_optical',
	    'cam_tx': 0.09, 'cam_ty': 0.0, 'cam_tz': -0.02,
	    'cam_roll_deg': 0.0, 'cam_pitch_deg': 0.0, 'cam_yaw_deg': 0.0,
	    # 关键三行：开启并指定输出 PoseStamped 到 /ego/camera_pose_ps
            'publish_camera_pose': True,
            'camera_pose_topic_out': '/ego/camera_pose_ps',
            'pose_frame_select': 'optical'  # 用 optical 相机帧发布位姿
	}]
    )
    # --------- 轨迹服务器 ----------
    # --------- 轨迹服务器 ----------
    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        # name = "drone_<id>_traj_server"
        name=[TextSubstitution(text='drone_'), drone_id, TextSubstitution(text='_traj_server')],
        output='screen',
        remappings=[
        # 'position_cmd'  ->  "/drone_<id>_planning/pos_cmd"
        ('position_cmd',   [TextSubstitution(text='/drone_'), drone_id, TextSubstitution(text='_planning/pos_cmd')]),
        # 'planning/bspline'  ->  "/drone_<id>_planning/bspline"
        ('planning/bspline',[TextSubstitution(text='/drone_'), drone_id, TextSubstitution(text='_planning/bspline')]),
    ],
        parameters=[{'traj_server/time_forward': 1.0}],
)
    # --------- RViz（可选，你已有单独 launch 的话可以移除） ----------
    # rviz = Node(
    #     package='rviz2', executable='rviz2', name='ego_rviz', output='screen',
    #     arguments=['-d', os.path.join(get_package_share_directory('ego_planner'), 'launch', 'default.rviz')]
    # )

    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    # 真实链路固定起（advanced_param、轨迹、poscmd_2_odom）
    ld.add_action(advanced_param)
    ld.add_action(traj_server)
    ld.add_action(poscmd_node)

    # 仅仿真时起模拟地图/渲染
    ld.add_action(map_generator)
    ld.add_action(pcl_render)
    ld.add_action(tf_node)
    return ld

